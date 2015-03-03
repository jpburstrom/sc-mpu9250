#ifndef _debugging_NIME_debug_h
#define _debugging_NIME_debug_h

#include <iostream>

#define debug(m) ;;

#ifndef NDEBUG
  #undef debug
  #define debug(msg) auto _debug_msg = InOutWrapper(msg);
#endif


// helpful debugging thing
struct InOutWrapper {
  InOutWrapper(const std::string& m)
    :
    msg(m)
  {
    std::cout << msg << " entered" << std::endl;
  }

  ~InOutWrapper() {
    std::cout << msg << " complete" << std::endl;
  }

  const std::string msg;
};


#endif
