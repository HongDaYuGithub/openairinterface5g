#ifndef ___HAVE_CONFIG_H_
#define ___HAVE_CONFIG_H_

// This is hard coded name "config.h" and HAVE_CONFIG_H_ in ans1c skeletons
#include "common/utils/LOG/log.h"
#if DEBUG_ASN1C
#define ASN_DEBUG(x...) do{LOG_I(ASN,x);LOG_I(ASN,"\n");}while(false)
#else 
#define ASN_DEBUG(x...)
#endif

#endif
