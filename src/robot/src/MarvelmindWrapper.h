#ifndef MarvelmindWrapper_h
#define MarvelmindWrapper_h

#include <vector>

extern "C" {
    #include <marvelmind/marvelmind.h>
}

class MarvelmindWrapper
{
  public:
    MarvelmindWrapper();
    ~MarvelmindWrapper();
    std::vector<float> getPositions();

  private:
    MarvelmindHedge* hedge_;
    bool ready_;

};


#endif
