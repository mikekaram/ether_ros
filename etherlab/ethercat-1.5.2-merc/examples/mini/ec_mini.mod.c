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
	{ 0x65891794, __VMLINUX_SYMBOL_STR(ecrt_master_receive) },
	{ 0xd2b09ce5, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0xf7360ef9, __VMLINUX_SYMBOL_STR(ecrt_master_create_domain) },
	{ 0x31ff2a16, __VMLINUX_SYMBOL_STR(ecrt_master_send) },
	{ 0x373e4c41, __VMLINUX_SYMBOL_STR(ecrt_domain_queue) },
	{ 0x9d7b8ab9, __VMLINUX_SYMBOL_STR(ecrt_master_send_ext) },
	{ 0x8fdf772a, __VMLINUX_SYMBOL_STR(init_timer_key) },
	{ 0x7341ba7b, __VMLINUX_SYMBOL_STR(ecrt_domain_reg_pdo_entry_list) },
	{ 0x7d11c268, __VMLINUX_SYMBOL_STR(jiffies) },
	{ 0x188ecad2, __VMLINUX_SYMBOL_STR(ecrt_domain_process) },
	{ 0x700a664d, __VMLINUX_SYMBOL_STR(ecrt_domain_state) },
	{ 0x3f5c6471, __VMLINUX_SYMBOL_STR(del_timer_sync) },
	{ 0xa50e8c08, __VMLINUX_SYMBOL_STR(ecrt_master_callbacks) },
	{ 0xf1e0ec09, __VMLINUX_SYMBOL_STR(ecrt_master_state) },
	{ 0x63b39dc9, __VMLINUX_SYMBOL_STR(ecrt_slave_config_state) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x54e68362, __VMLINUX_SYMBOL_STR(ecrt_master_slave_config) },
	{ 0x6dc6dd56, __VMLINUX_SYMBOL_STR(down) },
	{ 0x85825898, __VMLINUX_SYMBOL_STR(add_timer) },
	{ 0xb0f2d9c4, __VMLINUX_SYMBOL_STR(ecrt_slave_config_pdos) },
	{ 0xe4a91862, __VMLINUX_SYMBOL_STR(ecrt_domain_external_memory) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0xc62d2079, __VMLINUX_SYMBOL_STR(ecrt_domain_size) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x11b4837, __VMLINUX_SYMBOL_STR(ecrt_master_activate) },
	{ 0x2b9c2956, __VMLINUX_SYMBOL_STR(ecrt_release_master) },
	{ 0x78e739aa, __VMLINUX_SYMBOL_STR(up) },
	{ 0x77d5fc11, __VMLINUX_SYMBOL_STR(ecrt_request_master) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=ec_master";


MODULE_INFO(srcversion, "7D79A65FE0B0BD05CFF427C");
