
# install the necessary packages for building EtherLab
sudo apt-get install autoconf automake libtool mercurial

# hg clone might fail because there is no user registered. In this case uncomment and run the following line:
# echo -e '[extensions] \n mq = \n [ui] \n username = Foo Bar <foobar@mail.com>' > ~/.hgrc

# clone the EtherLab repository
hg clone -u 33b922ec1871 http://hg.code.sf.net/p/etherlabmaster/code ethercat-1.5.2-merc

# clone the patches
hg clone http://hg.code.sf.net/u/uecasm/etherlab-patches ethercat-1.5.2-merc/.hg/patches
cd ethercat-1.5.2-merc

# apply the patches
hg qpush -a
cd ..; make ethercatMasterInstallWithAutoStart
rm -rf ethercat-1.5.2-merc
