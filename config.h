//
// Created by Administrator on 2024/9/15.
//

#ifndef CONFIG_H
#define CONFIG_H


#define DEBUG 1
#if (DEBUG == 1)//开启静态，用于watch
        #define dstatic(type, name) static type name; name = 0 //赋值为0，实现伪局部，为了用来与static区别，不过这样会导致在绘图的时候随机刷新丝线，这是因为变0了
#elif (DEBUG == 0)
        #define dstatic(type, name) type name


#endif

#endif //CONFIG_H

