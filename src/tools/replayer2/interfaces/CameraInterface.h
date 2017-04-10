//
// Created by frivas on 11/04/17.
//

#ifndef JDEROBOT_CAMERAINTERFACE_H
#define JDEROBOT_CAMERAINTERFACE_H


#include <camera.h>
#include <logger/Logger.h>

class CameraInterface: virtual public jderobot::Camera {
public:
    CameraInterface(std::string& propertyPrefix, Ice::CommunicatorPtr ic, long long int initStateIN): prefix(propertyPrefix) {
        LOG(INFO)<< "Creating " + propertyPrefix;

        imageDescription = (new jderobot::ImageDescription());
        prop = ic->getProperties();
        cameraDescription = (new jderobot::CameraDescription());
        startThread = false;
        this->width=prop->getPropertyAsIntWithDefault(propertyPrefix + "ImageWidth",320);
        this->height=prop->getPropertyAsIntWithDefault(propertyPrefix + "ImageHeight",240);
        this->dataPath=prop->getProperty(propertyPrefix+"Dir");
        this->fileFormat=prop->getProperty(propertyPrefix+"FileFormat");
        this->format = prop->getProperty(propertyPrefix+"Format");
        LOG(INFO)<< "PATH " + this->dataPath ;
        LOG(INFO)<< "FORMAT: " + this->fileFormat ;


        this->initState=initStateIN;
        //sync task
        syncTask = new SyncTask(this,this->dataPath, this->fileFormat);
        syncTask->start();
        //reply task
        replyTask = new ReplyTask(this);
        replyTask->start(); // my own thread

    }

    std::string getName () {
        return (cameraDescription->name);
    }

    std::string getRobotName () {
        return (cameraDescription->name);

    }

    virtual ~CameraI() {
        LOG(INFO)<< "Stopping and joining thread for camera: " +  cameraDescription->name;
    }

    virtual jderobot::ImageDescriptionPtr getImageDescription(const Ice::Current& c){
        return imageDescription;
    }

    virtual jderobot::CameraDescriptionPtr getCameraDescription(const Ice::Current& c){
        return cameraDescription;
    }

    virtual Ice::Int setCameraDescription(const jderobot::CameraDescriptionPtr &description, const Ice::Current& c) {
        return 0;
    }

    virtual void getImageData_async(const jderobot::AMD_ImageProvider_getImageDataPtr& cb, const std::string& format, const Ice::Current& c){
        replyTask->pushJob(cb);
    }

    virtual jderobot::ImageFormat getImageFormat(const Ice::Current& c)
    {
        jderobot::ImageFormat formats;

        formats.push_back(colorspaces::ImageRGB8::FORMAT_RGB8.get()->name);

        return formats;
    }

    virtual std::string startCameraStreaming(const Ice::Current&){
        LOG(INFO)<< "Should be made anything to start camera streaming: " + cameraDescription->name;
        return std::string("");
    }

    virtual void stopCameraStreaming(const Ice::Current&) {
        LOG(INFO)<< "Should be made anything to stop camera streaming: " + cameraDescription->name;
    }

    virtual void reset(const Ice::Current&)
    {
    }

    /*virtual void update(cv::Mat imageIn){
     imageIn.copyTo(this->image);
     //std::cout << "update" << endl;

     if(!startThread){
        startThread = true;
        replyTask = new ReplyTask(this);
           replyTask->start(); // my own thread
       }
   }*/

private:
    class SyncTask: public IceUtil::Thread{
    public:
        SyncTask(CameraI* camera, std::string pathIn, std::string fileFormatIN){
            this->mycamera=camera;
            this->path=pathIn;
            this->initiated=false;
            this->fileFormat=fileFormatIN;
            this->onPause=false;
        }
        ~SyncTask(){
        }
        virtual void run(){
            std::string line;
            std::string fileName(this->path + "cameraData.jde");
            std::ifstream myfile(fileName.c_str());
            if (!myfile.is_open())
                LOG(ERROR)<< "Error while trying to open: " + fileName;
            while(this->isAlive()){
                while ( myfile.good() ){
                    bool playing=controller->getPlay();

                    this->onPause=!playing;
                    while (!playing){
                        playing=controller->getPlay();
                        long long int pauseStatus= controller->getSyncTime();
                        if (pauseStatus != this->mycamera->initState){
                            this->mycamera->initState=pauseStatus;
                            break;
                        }
                        //check if w
                        //std::cout << "not playing" << std::endl;
                        usleep(10000);
                        continue;
                    }

                    if (this->onPause){
                        this->mycamera->initState=controller->getSyncTime();
                        myfile.close();
                        myfile.open(fileName.c_str());
                    }

                    getline (myfile,line);
                    std::istringstream sTemp(line);
                    long long int relative;
                    sTemp >> relative;
                    lastRelative=relative;


                    //tiempo para comprobar si vamos muy desacompasados y para rewind - forward
                    IceUtil::Time pretime = IceUtil::Time::now();
                    long long int checkState=(pretime.toMicroSeconds())/1000;



                    while((((relative) - (checkState - this->mycamera->initState ))<0)&&(myfile.good())){
                        //no hacemos nada, estamos fuera de tiempo tenemos que avanzar al siguiente frame
                        getline (myfile,line);
                        std::istringstream sTemp(line);
                        sTemp >> relative;
                    }
                    if (!myfile.good()){
                        if (this->onPause)
                            continue;
                        else
                            break;
                    }


                    cv::Mat tempImage=cv::imread(this->path + line + "." + this->fileFormat);

                    IceUtil::Time a = IceUtil::Time::now();
                    long long int actualState=(a.toMicroSeconds())/1000;
                    /*std::cout << "Relativo fichero: " << relative << std::endl;
                    std::cout << "Relativo global: " << (actualState - this->mycamera->initState ) << std::endl;
                    std::cout << "Duermo: " << (relative) - (actualState - this->mycamera->initState ) << std::endl;*/
                    if ((actualState - this->mycamera->initState ) < relative ){
                        usleep(((relative) - (actualState - this->mycamera->initState ))*1000);
                    }


                    this->mycamera->dataMutex.lock();

                    tempImage.copyTo(this->mycamera->image);



                    /*cv::imshow("lector", this->mycamera->image);
                    cv::waitKey(0);*/
                    this->mycamera->dataMutex.unlock();

                }
                myfile.close();
                //control.controlMutex.lock();
                this->mycamera->initState=controller->wait();
                myfile.open(fileName.c_str());

            }

        }
    private:
        std::string path;
        CameraI* mycamera;
        bool initiated;
        std::string fileFormat;
        bool onPause;
        long long int lastRelative;
    };


    class ReplyTask: public IceUtil::Thread{
    public:
        ReplyTask(CameraI* camera){
            this->mycamera=camera;
        }

        void pushJob(const jderobot::AMD_ImageProvider_getImageDataPtr& cb){
            IceUtil::Mutex::Lock sync(requestsMutex);
            requests.push_back(cb);
        }

        virtual void run(){
            mycamera->imageDescription->width = this->mycamera->width;
            mycamera->imageDescription->height = this->mycamera->height;
            mycamera->imageDescription->size = this->mycamera->width*this->mycamera->height*3;
            mycamera->imageDescription->format = this->mycamera->format;

            jderobot::ImageDataPtr reply(new jderobot::ImageData);
            reply->description = mycamera->imageDescription;
            IceUtil::Time a, b;
            int cycle = 48;
            long totalb,totala;
            long diff;

            while(this->isAlive()){
                a = IceUtil::Time::now();
                totala=a.toMicroSeconds();


                IceUtil::Time t = IceUtil::Time::now();
                reply->timeStamp.seconds = (long)t.toSeconds();
                reply->timeStamp.useconds = (long)t.toMicroSeconds() - reply->timeStamp.seconds*1000000;

                reply->pixelData.resize(mycamera->imageDescription->size);

                //image = cv::imread(mycamera->fileName);
                this->mycamera->dataMutex.lock();
                if (this->mycamera->image.rows != 0){
                    memcpy( &(reply->pixelData[0]), (unsigned char *) this->mycamera->image.data, this->mycamera->image.rows*this->mycamera->image.cols*3);
                }

                this->mycamera->dataMutex.unlock();
                { //critical region start
                    IceUtil::Mutex::Lock sync(requestsMutex);
                    while(!requests.empty()) {
                        jderobot::AMD_ImageProvider_getImageDataPtr cb = requests.front();
                        requests.pop_front();
                        cb->ice_response(reply);
                    }
                } //critical region end

                b = IceUtil::Time::now();
                totalb=b.toMicroSeconds();

                diff = (totalb-totala)/1000;
                diff = cycle-diff;

                if(diff < 33)
                    diff = 33;


                /*Sleep Algorithm*/
                usleep(diff*1000);
            }
        }

        CameraI* mycamera;
        IceUtil::Mutex requestsMutex;
        std::list<jderobot::AMD_ImageProvider_getImageDataPtr> requests;
    };

    typedef IceUtil::Handle<ReplyTask> ReplyTaskPtr;
    std::string prefix;
    colorspaces::Image::FormatPtr imageFmt;
    jderobot::ImageDescriptionPtr imageDescription;
    jderobot::CameraDescriptionPtr cameraDescription;
    ReplyTaskPtr replyTask;
    bool startThread;
    Ice::PropertiesPtr prop;
    cv::Mat image;
    IceUtil::Mutex dataMutex;
    IceUtil::Time ref;
    int width;
    int height;
    typedef IceUtil::Handle<SyncTask> SyncTaskPtr;
    SyncTaskPtr syncTask;
    long long int initState;
    std::string fileFormat;
    std::string dataPath;
    std::string format;

}; // end class CameraI



#endif //JDEROBOT_CAMERAINTERFACE_H
