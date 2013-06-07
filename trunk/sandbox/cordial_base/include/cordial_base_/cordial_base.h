#ifndef CORDIAL_BASE__CORDIAL_BASE_H_
#define CORDIAL_BASE__CORDIAL_BASE_H_

#include <string>

namespace cordial_base
{
  class CordialBase
  {
    public:
      virtual void initialize() = 0;
      virtual std::string robot_name() = 0;

      


      virtual ~CordialBase(){}

    protected:
      CordialBase(){}
  };
}
#endif

