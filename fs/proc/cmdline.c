#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/setup.h>

static char proc_cmdline[COMMAND_LINE_SIZE];

static int cmdline_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", proc_cmdline);
	return 0;
}

static int cmdline_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cmdline_proc_show, NULL);
}

static const struct file_operations cmdline_proc_fops = {
	.open		= cmdline_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_cmdline_init(void)
{
	char *arg;
	arg = strstr(saved_command_line, "androidboot.verifiedbootstate=");
	if (arg) {
		char *eq = strchr(arg, '=');
		char *sp = strchr(arg, ' ');
		if (!sp)
			sp = "";
		scnprintf(proc_cmdline, sizeof(proc_cmdline), "%.*sgreen%s",
				(int)(eq - saved_command_line + 1),
				saved_command_line, sp);
	}
	else {
		strncpy(proc_cmdline, saved_command_line, sizeof(proc_cmdline));
	}
	proc_create("cmdline", 0, NULL, &cmdline_proc_fops);
	return 0;
}
fs_initcall(proc_cmdline_init);
