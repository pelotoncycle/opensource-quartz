1. Install repo tool:
	$ mkdir ~/bin -p
	$ sudo apt-get install curl
	$ curl https://dl-ssl.google.com/dl/googlesource/git-repo/repo > ~/bin/repo
	$ chmod a+x ~/bin/repo
	$ export PATH=~/bin:$PATH



2. Get Android Filesystem Sources:
	Open a terminal and move into the folder that you want to download Android source code.
	$ repo init -u git://git.omapzoom.org/platform/omapmanifest.git -b 27.x -m RLS4AJ.1.1_JellyBean.xml
	$ repo sync



3. The "external" folder includes the 3 modules which are modified for Quartz.
	Copy the modules to original Android source code.
	If the same module exists in Android source code, you should replace it.



4. For additional information, please visit the web page:
	http://www.omappedia.org/wiki/4AJ.1.1_OMAP4_Jelly_Bean_Release_Notes




