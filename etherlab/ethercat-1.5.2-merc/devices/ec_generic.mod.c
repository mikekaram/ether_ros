#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x683cfe8d, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x7875fbf2, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0x3ebefd9b, __VMLINUX_SYMBOL_STR(ecdev_open) },
	{ 0x6c506af3, __VMLINUX_SYMBOL_STR(ecdev_withdraw) },
	{ 0xeefe2ab8, __VMLINUX_SYMBOL_STR(kernel_sendmsg) },
	{ 0xe9d18093, __VMLINUX_SYMBOL_STR(sock_release) },
	{ 0x179651ac, __VMLINUX_SYMBOL_STR(_raw_read_lock) },
	{ 0x9eaf4194, __VMLINUX_SYMBOL_STR(dev_base_lock) },
	{ 0xf919aad3, __VMLINUX_SYMBOL_STR(sock_create_kern) },
	{ 0x3da2c7f, __VMLINUX_SYMBOL_STR(ecdev_offer) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x3fc0ec76, __VMLINUX_SYMBOL_STR(free_netdev) },
	{ 0x9166fada, __VMLINUX_SYMBOL_STR(strncpy) },
	{ 0xf5a1a046, __VMLINUX_SYMBOL_STR(init_net) },
	{ 0xb433ea34, __VMLINUX_SYMBOL_STR(ecdev_receive) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0x6cf65ab7, __VMLINUX_SYMBOL_STR(ecdev_set_link) },
	{ 0x870e13d3, __VMLINUX_SYMBOL_STR(alloc_netdev_mqs) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
	{ 0x3b475b41, __VMLINUX_SYMBOL_STR(ether_setup) },
	{ 0xccca000a, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0xfe8030b3, __VMLINUX_SYMBOL_STR(kernel_recvmsg) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x42da6677, __VMLINUX_SYMBOL_STR(kernel_bind) },
	{ 0xd9b5f551, __VMLINUX_SYMBOL_STR(ecdev_close) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=ec_master";


MODULE_INFO(srcversion, "1AE486D6201500C92B67C63");
