sudo apt-get install autoconf automake libtool mercurial
echo '[extensions]\nmq = \n' >> ~/.hgrc
hg clone -u 33b922ec1871 http://hg.code.sf.net/p/etherlabmaster/code ethercat-1.5.2-merc
hg clone http://hg.code.sf.net/u/uecasm/etherlab-patches ethercat-1.5.2-merc/.hg/patches
cd ethercat-1.5.2-merc
hg qpush -a
cd ..; make ethercatMasterInstallWithAutoStart
