#使用cubemx重建工程后需要完成以下步骤：
1.删除it文件中的重复定义的中断处理函数；
2.开启use Microlib；
3.INCLUDE_uxTaskGetStackHighWaterMark 0->1;
4.INCLUDE_xTaskGetHandle 0->1;