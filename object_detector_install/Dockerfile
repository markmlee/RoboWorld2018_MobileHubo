FROM nvidia/cudagl:9.0-devel-ubuntu16.04

RUN apt-get update -y --fix-missing 
RUN apt-get upgrade -y
RUN apt-get autoremove -y

# Text editors
RUN apt-get install -y --no-install-recommends\
	gedit \
	vim \ 
	wget
# Install sublime 
RUN wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | apt-key add - && \
	echo "deb https://download.sublimetext.com/ apt/stable/" | tee /etc/apt/sources.list.d/sublime-text.list
RUN apt-get update -y && \
	apt-get install -y --no-install-recommends sublime-text


###########################################
###			ROS KINETIC    copy & paste ###
###########################################
# install packages
RUN apt-get update && \
	apt-get install -q -y --no-install-recommends\
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y --no-install-recommends\
    ros-kinetic-ros-core=1.3.2-0* \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
#--------------------------------------------------
# custom packages for ROS

# install ros packages (kinetic-ros-base) -- stuff including visulaization stuff
RUN apt-get update && apt-get install -y --no-install-recommends\
    ros-kinetic-ros-base=1.3.2-0* \
	&& rm -rf /var/lib/apt/lists/*

# install ros packages (opencv3)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kinetic-opencv3 \
    ros-kinetic-vision-opencv \
	&& rm -rf /var/lib/apt/lists/*


###########################################
###			Catkin_workspace	    	###
###########################################

RUN mkdir -p /home/hubo/catkin_ws/src 
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && \
	cd /home/hubo/catkin_ws/ && \
	catkin_make"


#######################################################
###	 	Numpy, Cython, Pytorch, Scipy, Matplotlib 	###
#######################################################

# Install pip, numpy & cython 
RUN wget --quiet https://bootstrap.pypa.io/get-pip.py && \
	python get-pip.py 
RUN pip2 install --upgrade pip
RUN pip2 install cython==0.25.2
RUN pip2 install numpy

# Pytorch
RUN pip2 install http://download.pytorch.org/whl/cu90/torch-0.3.1-cp27-cp27mu-linux_x86_64.whl
RUN pip2 install torchvision
RUN pip2 install cffi

# Add scipy, matplotlib
RUN pip2 install scipy
RUN pip2 install matplotlib


############################################
####				PCL 1.8				 ###
############################################

RUN apt-get update 
RUN apt-get install -y --no-install-recommends\
	software-properties-common \
  	ca-certificates \
	curl \
	git  \
	python3-dev \
	mesa-utils \
	g++ \
	cmake \
	cmake-gui \
	doxygen \
	mpi-default-dev \
	openmpi-bin \
	openmpi-common \
	libusb-1.0-0-dev \
	libqhull* \
	libusb-dev \
	libgtest-dev \
	git-core \
	freeglut3-dev \
	pkg-config \
	build-essential \
	libxmu-dev \
	libxi-dev \
	libphonon-dev \
	libphonon-dev \
	phonon-backend-gstreamer \
	phonon-backend-vlc \
	graphviz \
	mono-complete \
	qt-sdk \
	libflann-dev \
	libflann1.8 \
	libboost1.58-all-dev \
	&& rm -rf /var/lib/apt/lists/*

############ Install PCL v 1.8 ############

# Source https://askubuntu.com/questions/916260/how-to-install-point-cloud-library-v1-8-pcl-1-8-0-on-ubuntu-16-04-2-lts-for#
# Add the JDK 8 and accept licenses (mandatory)
# Source https://gist.github.com/teekaay/39bff8c66cd8e43c2ba57a6b2eef4fa8
RUN add-apt-repository ppa:webupd8team/java && \
    echo debconf shared/accepted-oracle-license-v1-1 select true | debconf-set-selections && \
	echo debconf shared/accepted-oracle-license-v1-1 seen true | debconf-set-selections
RUN add-apt-repository -y ppa:webupd8team/java && \
	apt-get update && \
	apt-get --yes --no-install-recommends install oracle-java8-installer
RUN wget --quiet http://launchpadlibrarian.net/209530212/libeigen3-dev_3.2.5-4_all.deb && \
	dpkg -i libeigen3-dev_3.2.5-4_all.deb && \
	rm libeigen3-dev_3.2.5-4_all.deb
RUN apt-mark hold libeigen3-dev
RUN wget --quiet http://www.vtk.org/files/release/7.1/VTK-7.1.0.tar.gz && \
	tar -xf VTK-7.1.0.tar.gz && \
	rm VTK-7.1.0.tar.gz
RUN cd VTK-7.1.0 && \
	mkdir build && \
	cd build && \
	cmake .. && \
	make && \
	make install
RUN wget --quiet https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.0.tar.gz && \
	tar -xf pcl-1.8.0.tar.gz && \
	rm pcl-1.8.0.tar.gz
RUN cd pcl-pcl-1.8.0 && \
	mkdir build && \
	cd build && \
	cmake .. && \
	make && \
	make install
RUN rm -r /VTK-7.1.0 && \
	rm -r /pcl-pcl-1.8.0


# Clone python-pcl repo
RUN git clone https://github.com/strawlab/python-pcl.git
# Modify setup.py
RUN cd python-pcl && \
	sed -i "s/# ext_args\['include_dirs'\]\.append('\/usr\/include\/vtk-5\.8')/ext_args\['include_dirs'\]\.append('\/usr\/local\/include\/vtk-7\.1')/g" setup.py && \
	sed -i "s/# ext_args\['library_dirs'\]\.append('\/usr\/lib')/ext_args\['library_dirs'\]\.append('\/usr\/lib')/g" setup.py && \
	sed -i "s/# Extension(\"pcl.pcl_visualization\"/Extension(\"pcl.pcl_visualization\"/g" setup.py
RUN cd python-pcl && \
	python2 setup.py build_ext -i && \
	python2 setup.py install


############################################
####				USER 				 ###
############################################

ENV HOME /home/hubo
WORKDIR /home/hubo

# Set HUBO specific ros network setting
RUN echo 'source /home/hubo/catkin_ws/devel/setup.bash' >> /.bashrc
RUN echo 'export ROS_MASTER_URI=http://vision_pc:11311' >> /.bashrc
RUN echo 'export ROS_IP=10.12.3.5' >> /.bashrc
RUN echo 'export ROS_IP=10.12.3.20' >> /.bashrc
