

# 华东理工大学课程攻略共享计划

![1574474713395](readme.assets/1574474713395.png)

![size](https://img.shields.io/github/repo-size/tianyilt/ecust-CourseShare)

本项目以[浙江大学课程攻略共享计划](https://github.com/QSCTech/zju-icicles )为模板


## 这个项目是什么

本项目是为了打破课程壁垒, 在这里你可以找到各种课程的资料. 

## 我应该如何使用
如果你现在使用手机,强烈建议使用电脑端浏览
点击上方**View Code**按钮可以看到一个类似百度网盘的页面

* 我想[下载一部分文件](./guide-and-notice/下载一份文件.md)
* 我想[上传一份文件](./guide-and-notice/上传一份文件.md)
* 我想[长期合作](./guide-and-notice/长期合作.md)

* 更加方便快捷地下载
 由于资料库比较大,可以先通过百度网盘链接下载,然后进行同步

 网盘链接如下:(辣鸡网盘,屏蔽我的分享,有什么其他好方法请issue告知我)

```
链接：https://pan.baidu.com/s/1toHeEz0oMvN1H5lXhZrCHw 
提取码：f2fm 
最近更新时间：2020.5.14
```

除了百度网盘之外还可以通过GitHub的镜像来快速克隆，预计半小时就可以所有资料下载到本地，然后配合everything等文件检索工具来查找需要的资料。

正确安装git工具后，在命令行中运行：

```
git clone --depth=1 https://hub.fastgit.xyz/tianyilt/ecust-CourseShare.git
```
通过白嫖一些同学的大创经费，目前有一个可以快速访问的[Alist](https://alist.世界百流大学.com/ecust-CourseShare)镜像，同时提供一部分教材的下载 。访问地址：

```
https://alist.世界百流大学.com/ecust-CourseShare
```

想要快速浏览整个项目的文件结构可以访问 :

```
https://github1s.com/tianyilt/ecust-CourseShare
```




## 前言

- 我们非常重视版权保护,如有侵权内容,请在issues下指出,我们将会快速删除,非常感谢.
- 引用[浙江大学课程攻略共享计划](https://github.com/QSCTech/zju-icicles )的前言:

> 来到一所大学，从第一次接触许多课，直到一门一门完成，这个过程中我们时常收集起许多资料和情报。
>
> 有些是需要在网上搜索的电子书，每次见到一门新课程，Google 一下教材名称，有的可以立即找到，有的却是要花费许多眼力；有些是历年试卷或者 A4 纸，前人精心收集制作，抱着能对他人有用的想法公开，却需要在各个群以至于从学长学姐手中代代相传；有些是上完一门课才恍然领悟的技巧，原来这门课重点如此，当初本可以更轻松地完成得更好……
>
> 我也曾很努力地收集各种课程资料，但到最后，某些重要信息的得到却往往依然是纯属偶然。这种状态时常令我感到后怕与不安。我也曾在课程结束后终于有了些许方法与总结，但这些想法挂在群相册,局限于一个年级，最终只能把花费时间与精力才换来的经验耗散在了漫漫的遗忘之中。
>
> 我为这一年一年，这么多人孤军奋战的重复劳动感到不平。
>
> 我希望能够将这些隐晦的、不确定的、口口相传的资料和经验，变为公开的、易于获取的和大家能够**共同完善、积累**的共享资料。
>
> 我希望只要是前人走过的弯路，后人就不必再走。这是我的信念，也是我建立这个项目的原因。

- 整个课程攻略共享计划分为3阶段，
  - 第一阶段qq群，完成了，但是年级之间交流很少，资料传着传着就没了，此外没法保留文件结构，压缩包居多，群相册存储资料也很麻烦。
  - 第二阶段网盘，问题出在如果要永世更新，多人共同编辑不方便，无法控制版本，长期更新很依赖于单个管理员。
  - 第三阶段，采用git管理，并选择github平台，也就是当前在进行的项目。原因如下：
      - 受到github上众多相关项目的启发
      - GitHub 项目可以使用目录进行文件组织，并且每个目录均可以在显示文件列表的同时显示一个 README，十分适合知识的传承与发展。
      - GitHub 带有便捷的 Issue（类似于提意见的论坛） 和 Pull Request（自己修改内容，然后提出请求，最后合并） 协作功能，并且可以方便地对贡献的质量进行监督和调整。
      
## 在这里可以找到什么

* 学院的全称（相信我，真的有小可爱不知道自己学院叫什么的）
* [公共课](/A公共课导引):如[高等数学](/数学学院/公共课/高数)、[线性代数](/数学学院/公共课/线性代数)、[大学物理](/物理学院/公共课)等
* [化学院](/化学与分子工程学院)的完整资料
* [信院](/信息科学与工程学院)的完整资料
* [物理学院](/物理学院)的完整资料
* [数学学院](/数学学院)的完整资料
* [化工学院](/化工学院)的部分资料
* [材料学院](/材料科学与工程学院)的部分资料
* 以及一些零散、亟待补充的内容。

## 2022.12 文件树重构

华理有众多专业共享同一门课程，也有不同学院开设的同名课程。故进行了文件树的重构，完全依照开课学院进行分组，在其他学院的readme中加入超链接来进行跳转。

* 如果多个学院共享同一门课程，如化工学院、化分学院等都有化工原理课程，请全部归于单一开课学院，也就是[/化工学院/化工原理](/化工学院/化工原理)中。
* 如果不同学院开设的同名课程，如由数学学院和信息学院分别开设的计算机网络，请分别归于开课学院，如[/数学学院/计算机网络](/数学学院/计算机网络) 和[/信息科学与工程学院/计算机网络](/信息科学与工程学院/计算机网络) 中。建议在readme中加入相互的跳转。
* 对于同名的不同学分课程，我们建议在文件夹名中备注学分。
* 公共课程设有专门的文件夹用来导航。
* 建议各位贡献者们都在上方的“在这里可以找到什么”中留下自己的昵称或GitHub账号。

课程开课学院完全按照[教务处公示](https://jwc.ecust.edu.cn/2022/0919/c3938a148491/page.htm) *<font color="red">更新于2022.12 2023春季学期开学请更新</font>* 。



<font size="12">！！！郑重警告，抄作业是没有前途的</font> 

****



## 内容

本项目计划收录了以下内容：

- TODO选课攻略
- 电子版教材（提供下载外链以节省项目空间）
- 平时作业答案
- 历年试卷
- 复习资料
- TODO开卷考试 A4 纸
- 教师课件



## 教材（代补完）
请到学校[课程网站](http://e-learning.ecust.edu.cn/Portal/CC#Index)查询教材

## TODOlist（亟待修改） 
TODOlist表示计划进一步撰写、细化的部分
- [ ] 向没有git基础的广大同学介绍简便使用方式
    - [x] 下载
    - [X] 同步 百度网盘连着git下载**想clone的同学可以先在百度网盘上把库连着.git文件夹一起下载下来.然后修改上游为fork的仓库,然后fetch一下.百度网盘被设置为备份盘,理论上是保持一致的,有不一致就pull一下,以上操作在github desktop中轻松搞定**
    - [ ] 贡献 拉个群然后口口相传中，毕竟这个git学习成本放那里。
- [x] 移动设备中简单使用优化
    - [x] 百度网盘使用
    - [ ] 安利app
    - [x] NAS的链接真好用(`http://gofile.me/6KieW/ezEKFle3g`)
- [x] 解决github下载速度感人至深 
    - [x] 买20M校园网
    - [x] 百度网盘
    - [x] 硬盘直拷
- [x] 解决单独部分文件下载的问题
    - [x] 在网页上用[downgit](http://zhoudaxiaa.gitee.io/downgit/#/home)下载
    - [x] 百度网盘
    - [x] 用软件TortoiseSVN下载
- [ ] 制作项目文档，形成作品网站,参考[中科大的项目](https://ustc-resource.github.io/USTC-Course/)
- [x] 邀请志同道合者,
- [ ] **形成项目传承长效机制，每一位来到学校的同学都能享受历年传承的攻略并可以贡献自己的智慧。**
- [ ] **项目瘦身！！！！太 大 了！！！**

## 贡献规范

***非常欢迎贡献!***

***非常欢迎贡献!***

***非常欢迎贡献!***

Issue、PR、纠错、资料、选课/考试攻略，完全欢迎！

来自大家的关注、维护和贡献，才是让这个攻略共享计划存在的动力~

贡献需要注意如下几点:

- 本仓库未启用`git-lfs`，请不要上传单个超过100M的文件，否则commit无效。
- 参考[文件结构](./guide-and-notice/文件结构.md)进行贡献,如果有建议,欢迎开个issue讨论.
- 对于教师的评价请一律使用姓名拼音首字母缩写.

## Reference

* [清华大学计算机系课程攻略](https://github.com/PKUanonym/REKCARC-TSC-UHT)
* [浙江大学课程攻略共享计划](https://github.com/QSCTech/zju-icicles )
* [中国科技大学课程攻略共享计划](https://ustc-resource.github.io/USTC-Course/)
* [上海交通大学求生手册（雾）](https://github.com/SurviveSJTU/SurviveSJTUManual)

## 传送门

以下是非本仓库但是其他极客同学贡献的一些关于课程的仓库.
* !!!!华东理工大学飞跃手册:[github仓库](https://github.com/paulzrq/Ecust-Leap)
* 大物实验自动化脚本:[github仓库](https://github.com/ff6757442/experiment_kit)
* [马原答题程序](https://github.com/YifeiYang210/2020ecustMAYUAN)
* [计131资源共享站](https://www.6ccloud.com/)

## 致谢名单
- 16级：以PanJR XieYJ LuLW ChenGR TaoZH为代表的奆佬学长学姐帮助（现在不知道奆佬们的Github用户名，所以用姓拼音加名首字母代指）
- 17级：互相帮助合作 a.e.
- 18级-0xffffffff级:欢迎每一个关注该项目的你们
- 20级：由[wu2305](https://github.com/wu2305) 和[Liz](https://github.com/Liz-Nozomi) 等同学重构。欢迎大家来补文件！

## 许可

[CC-BY-NC-SA：署名-非商业性使用-相同方式共享](https://creativecommons.org/licenses/by-nc-sa/4.0/deed.zh)

> 资料仅供参考，请自己判断其适用性。

其他部分的版权归属于其各自的作者。

