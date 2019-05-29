


############
##  TEAM  ##
############
Team name: FIGMENT
Affiliation: IFPE/UFPE/ERICSSON

Members:

Alexandre Fransisco: alexandre.santos@gprt.ufpe.br
Daniel Bezerra:      daniel.bezerra@gprt.ufpe.br 
Djamel Sadok:        jamel@gprt.ufpe.br
Géza Szabó:          geza.szabo@ericsson.com
Jairo Matheus:       jairo.matheus@gprt.ufpe.br 
Kessia Nepomuceno:   kessia.nepomuceno@gprt.ufpe.br
Marcos Machado:      marcos.machado@gprt.ufpe.br
Rafael Aschoff:      rafael.roque@palmares.ifpe.edu.br

#######################
## Remote Simulation ##
#######################
source /opt/ros/kinetic/setup.bash
export ROS_HOSTNAME=LOCAL_IP
export ROS_MASTER_URI=http://SERVER_IP:11311/
export GAZEBO_IP=LOCAL_IP
export GAZEBO_MASTER_URI=SERVER_IP:12345



##############
## GIT HELP ##
##############
Command line instructions
Git global setup

git config --global user.name "Full Name"
git config --global user.email "login@gprt.ufpe.br"

Create a new repository

git clone git@gitlab.gprt.ufpe.br:figment/ariac-final.git
cd ariac-final
touch README.md
git add README.md
git commit -m "add README"
git push -u origin master

Existing folder

cd existing_folder
git init
git remote add origin git@gitlab.gprt.ufpe.br:figment/ariac-final.git
git add .
git commit
git push -u origin master

Existing Git repository

cd existing_repo
git remote add origin git@gitlab.gprt.ufpe.br:figment/ariac-final.git
git push -u origin --all
git push -u origin --tags
