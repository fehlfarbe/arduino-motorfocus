#include <Stream.h>

class DummySerial : public Stream {

  public:
    void begin(long speed){
      
    }

    virtual size_t write(uint8_t byte){
      return 1;
    }
    virtual int read(){
      return 0;
    }
    virtual int available(){
      return 0;
    }
    virtual void flush(){
      
    }

    virtual int peek(){
      return 0;
    }
};
