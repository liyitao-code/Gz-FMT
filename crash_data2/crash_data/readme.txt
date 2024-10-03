这里面crash_ren是我从github上老师的issue复制下来的，random_1_crash是我这边纯随机方法在8.6.0跑24h得到的崩溃，random_2_crash是在8.0.0的结果，只需要关注里面的unique_crash文件夹里的就可以
crash_result.py是筛选用的脚本，里面有注释标记使用方法

检测崩溃是否相同的方法分几种，都是关注gz.err文件里的内容实现：

1、如果#0行有内容，则可以用这一行的函数签名来区分。例如：
random_1_crash/unique_crash/_128里的gz.err的内容是
Stack trace (most recent call last) in thread 1370988:
#6    Object "[0xffffffffffffffff]", at 0xffffffffffffffff, in 
#5    Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x73a0baf2684f, in 
#4    Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x73a0bae94ac2, in 
#3    Object "/lib/x86_64-linux-gnu/libstdc++.so.6", at 0x73a0b68dc252, in 
#2    Object "/lib/x86_64-linux-gnu/libgz-sim8.so.8", at 0x73a0b574bd5d, in 
#1    Object "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/libgz-sim-ackermann-steering-system.so", at 0x73a0a5cd782d, in non-virtual thunk to gz::sim::v8::systems::AckermannSteering::PostUpdate(gz::sim::v8::UpdateInfo const&, gz::sim::v8::EntityComponentManager const&)
#0    Object "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/libgz-sim-ackermann-steering-system.so", at 0x73a0a5ccf38c, in gz::sim::v8::systems::AckermannSteeringPrivate::UpdateVelocity(gz::sim::v8::UpdateInfo const&, gz::sim::v8::EntityComponentManager const&)
Segmentation fault (Address not mapped to object [(nil)])
这里#0行的函数签名是gz::sim::v8::systems::AckermannSteeringPrivate::UpdateVelocity(gz::sim::v8::UpdateInfo const&, gz::sim::v8::EntityComponentManager const&)，
random_2_crash/unique_crash/_57里也是这样，它们就是同一个crash，当然它们的报错栈其实是完全一样的

2、检测错误栈里的函数调用顺序，目前脚本里的方法就是这样的。例如：
random_1_crash/unique_crash/_128里的gz.err的内容是
Stack trace (most recent call last) in thread 1370988:
#6    Object "[0xffffffffffffffff]", at 0xffffffffffffffff, in 
#5    Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x73a0baf2684f, in 
#4    Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x73a0bae94ac2, in 
#3    Object "/lib/x86_64-linux-gnu/libstdc++.so.6", at 0x73a0b68dc252, in 
#2    Object "/lib/x86_64-linux-gnu/libgz-sim8.so.8", at 0x73a0b574bd5d, in 
#1    Object "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/libgz-sim-ackermann-steering-system.so", at 0x73a0a5cd782d, in non-virtual thunk to gz::sim::v8::systems::AckermannSteering::PostUpdate(gz::sim::v8::UpdateInfo const&, gz::sim::v8::EntityComponentManager const&)
#0    Object "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/libgz-sim-ackermann-steering-system.so", at 0x73a0a5ccf38c, in gz::sim::v8::systems::AckermannSteeringPrivate::UpdateVelocity(gz::sim::v8::UpdateInfo const&, gz::sim::v8::EntityComponentManager const&)
Segmentation fault (Address not mapped to object [(nil)])
错误栈就是：
('non-virtual thunk to gz::sim::v8::systems::AckermannSteering::PostUpdate(gz::sim::v8::UpdateInfo const&, gz::sim::v8::EntityComponentManager const&)', 'gz::sim::v8::systems::AckermannSteeringPrivate::UpdateVelocity(gz::sim::v8::UpdateInfo const&, gz::sim::v8::EntityComponentManager const&)')
其实random_2_crash/unique_crash/_57也是这样的，所以它们是一样的crash

3、目前可能存在问题，因为有一些错误栈的深度可能不一样，但是大体上的调用顺序是一样的，那么也认为是一样的crash。这个暂时没有具体的例子，需要人工筛选一下

任务1是帮我检测一下现在crash_result.py脚本的功能，目前我将两个随机方法的crash合并之后，尝试和crash_ren里的数据对比，寻找新的崩溃，但是结果显示全部都是新崩溃，而人工复核的结果是至少有好几个是一样的，所以应该是有问题的，需要修改，脚本改好后发我一份
任务2是在修好之后，检测一下随机方法里的crash有多少是crash_ren里没有的，包括使用脚本筛选后人工再筛一下



