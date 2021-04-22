# A.N.A.S Development Guide
## 1. Introduction

This is a repository for the development of A.N.A.S by Group 2 Studio 1 EG2310. In here, you will find the code for ANAS and all the design and iteration of ANAS. You will also find all the tool chain that we use and version update log.

## 2. Development Tools
### Visual Studio Code
VSCode is a lightweight and highly functional text editor and pseudo-IDE which is recommended for this project. You can find the installation guide here:
> [Visual Studio Code](https://code.visualstudio.com/)
### Git & Github
Collaboration in programming world would be impossible without one of mankind greatest invention **Git**. Git is a Source Code Management
It is necessary that all members are familiar with how to use **Git and Github** to do basic code management such as commit, pull, and push. You can find out how to install Git here:
> [Git](https://git-scm.com/)

On the same website, there is also a [book](https://git-scm.com/book/en/v2) for indepth exploration of what **Git** can do (TLDR: it can do **A LOT** of thing). Here is a joke for you: 
> How do you win a programming competition?
>  
> You need to "Git good"

For those who are starting out and a bit clueless about Git or those who already know what is Git but have trouble navigating around, here is some resources for you all:
* [Harvard Git Lecture](https://cs50.harvard.edu/web/2020/weeks/1/) 
* [Github & Git cheatsheet](https://education.github.com/git-cheat-sheet-education.pdf)

PS: although Github make life easier by putting GUI so you don't have to use CLI to execute Git command, I still recommend you to use Bash as it will let you understand Git better and you can easily do more complex stuff in the future.

### Powerline for Linux terminal
Powerline is a way to customize the terminal in Linux so that it looks nicer and easier to understand. It will help you a lot in making sense of Git and for navigating using CLI in general. Follow the instruction here to set up your ClI nicely:
> [Powerline setup](https://www.ricalo.com/blog/install-powerline-ubuntu/#install-powerline)

If you meet an error that does not show you the branch position you are on, please see the discussion here:
> [Fix branch status not showing](https://github.com/powerline/powerline/issues/186#issuecomment-247810572)

### Huy's personal recommendation (for an easier and stress free life)
You might want to download the following extensions on your VSCode (from the extension tab) which can do a lot of cool stuffs (eg. auto complete, header finding, intellisense, ...) that help manage your workflow and monitor other workflow, just type the key words as you see bellow:
* Git History
* GitLens
* C/C++
* Python
* TabNine (this is some cool sorcery stuff)
  
### Markdown
Want to know how I make this cool document? I written it in a markup language called Markdown (yeah I know ironically). You can find more information about it here:
> [Markdown](https://www.markdownguide.org/)

If you decide to pick up this cool documentation language and want to help with documenting, you can also download this following extensions in VSCode:
* Markdown preview
* Mardown All in One

## 3. Progress
### A.N.A.S  v1.0 (Wall Follower) (Deprecated)
> This is legacy code which is not saved
#### Logic
* Hugs right 
* Using rotatebot() and lidar data
* Minimum distance from the wall is 20cm
* Maximum distance from the wall is 30cm
* Everytime it is closer than 20cm it turns left 10<sup>o</sup>, if it is more than 30cm it turns right 10<sup>o</sup>
* After turning, move forward for 2 seconds
#### Problems:
* A lot of jitter
* Movement speed and turning is prety slow and inaccurate
* The forward movment is hardcode and not adaptive
* Cannot turn corner well and
### A.N.A.S  v2.0 (A*STAR Path Finding) (Unstable)
> To run this logic   
> `ros2 run anas_ai pos`  
> `ros2 run anas_ai nav`
#### Logic
* Based on Andreas Bresser [Pathfinding library](https://pypi.org/project/pathfinding/), we can generate a path (list of coordinates) to a certain coordinate given the map, the starting position and the ending position of the bot.  
* Using OpenCV Python Library CV2 and examine the map generated from occupancy msg. Highligh all the intersection between unexplored region and explroed region which have a clear path. (Run `ros2 run anas_ai occ` to see what the highligh would looks like).
* Find the furthest highlight point from the bot and move take that as the target.
* Input the target into the pathfinding library, get the list of path coordinate, and feed it into the bot.
#### Problems
* The turning and moving between coordinate takes very long time
* To complete one set of path takes more than 2 mintues
* The orientation of the map generated and the map is in wrong orientation regularly, making the rotatebot code very confusing for the bot, because it doesnt know where to go.

### A.N.A.S  v3.0 (Wall Follower v2) (Stable)
> To run this logic   
> `ros2 run anas_ai state`  
> `ros2 run anas_ai follow`
#### Logic
* Same logic but upgrade and use some more advanced code with reference to Bug2 Algorithm on [this webpage](https://automaticaddison.com/the-bug2-algorithm-for-robot-motion-planning/)
#### Problems
* Does not work really well when all the walls are disconnected
* The speed is still relatively slow, but acceptable





