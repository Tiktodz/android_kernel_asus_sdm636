#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/ctype.h>
#include <asm/setup.h>

static char new_command_line[COMMAND_LINE_SIZE];

static int cmdline_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", new_command_line);
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

static unsigned int arg_len(const char *arg)
{
	unsigned int i, quoted = 0;

	for(i = 0; arg[i] != '\0'; i++) {
		if (isspace(arg[i]) && !quoted)
			break;
		if (arg[i] == '"') quoted = !quoted;
	}
	return i;
}

static char * unquote(char *str)
{
	unsigned int len;

	if (*str == '"') str++;
	len = strlen(str);
	if ((len > 0) && (str[len - 1] == '"')) str[len - 1] = '\0';
	return str;
}

static int append(const char *str, unsigned int len)
{
	unsigned int cmd_len;

	cmd_len = strlen(new_command_line);
	if (cmd_len == 0) {
		if (len >= sizeof(new_command_line))
			return -1;
	} else {
		if (cmd_len + len + 1 >= sizeof(new_command_line))
			return -1;
		new_command_line[cmd_len++] = ' ';
	}
	strncat(new_command_line + cmd_len, str, len);
	new_command_line[cmd_len + len] = '\0';
	return 0;
}

static int modify_cmd_line(const char *cmd_line,
                           int (*callback)(const char *name, const char *val, const char ***list))
{
	static char	buf[COMMAND_LINE_SIZE];
	const char	*in, *name, *val, **list;
	char		*q;
	unsigned int	len;
	int		i, accept;

	in = cmd_line;
	*new_command_line = '\0';
	while(*in != '\0'){
		in = skip_spaces(in);
		len = arg_len(in);
		if (len >= sizeof(buf)) {
			pr_err("modify_cmd_line: parameter is too long, len=%d\n", len);
			goto error;
		}

		strncpy(buf, in, len);
		buf[len] = '\0';
		q = strchr(buf, '=');
		if (q != NULL) {
			*q = '\0';
			name = unquote(buf);
			val = unquote(q + 1);
		} else {
			name = NULL;
			val = unquote(buf);
		}

		list = NULL;
		accept = (callback == NULL) ? 1 : callback(name, val, &list);
		if (accept && (append(in, len) != 0)) {
			pr_err("modify_cmd_line: out string overflow\n");
			goto error;
		}
		for(i = 0; (list != NULL) && (list[i] != NULL); i++) {
			if (append(list[i], strlen(list[i])) != 0) {
				pr_err("modify_cmd_line: out string overflow\n");
				goto error;
			}
		}

		in += len;
	}

	if (callback != NULL) {
		list = NULL;
		callback(NULL, NULL, &list);
		for(i = 0; (list != NULL) && (list[i] != NULL); i++) {
			if (append(list[i], strlen(list[i])) != 0) {
				pr_err("modify_cmd_line: out string overflow\n");
				goto error;
			}
		}
	}
	return 0;

    error:
	strcpy(new_command_line, cmd_line);
	return -1;
}

/*
 * This function will be called for each of command line parameters and at the
 * end of patameter list as well. See modify_cmd_line_example_callback() below.
 */
static int modify_cmd_line_callback(const char *name, const char *val, const char ***list)
{
	(void)val;

	if ((name != NULL) &&
	    (strcmp(name, "androidboot.verifiedbootstate") == 0)) {
		// NULL terminated list of strings to replace "androidboot.verifiedbootstate"
		static const char *replace_list[] = { "androidboot.verifiedbootstate=green", NULL };
		*list = replace_list;
		return 0; /* drop old value */
	}

	return 1; /* accept by default */
}

#if 0
/*
 * Example callback:
 * This function will be called for each of command line parameters and at the
 * end of patameter list as well.
 */
static int modify_cmd_line_example_callback(const char *name, const char *val, const char ***list)
{
	/* check for end of parameter list */
	if ((name == NULL) && (val == NULL)){
		// NULL terminated list of strings to append at the end of command line
		static const char *return_list[] = { "some_param1=some_value1", "simple_param1", NULL };
		*list = return_list;
		return 1;
	}

	/* check for unnamed papameters */
	if ((name == NULL) && (val != NULL)) {
		return 1; /* accept */
	}

	/* drop parameters */
	if ((strcmp(name, "androidboot.enable_dm_verity")  == 0) ||
	    (strcmp(name, "androidboot.secboot")           == 0) ||
	    (strcmp(name, "androidboot.verifiedbootstate") == 0) ||
	    (strcmp(name, "androidboot.veritymode")        == 0)) {
		return 0; /* drop */
	}

	/* replace parameters */
	if (strcmp(name, "some_param2") == 0) {
		// NULL terminated list of strings to replace "some_param2"
		static const char *return_list[] = { "other_param1=other_value1", "other_param2=other_value2", NULL };
		// protection against double replacent
		static int replaced = 0;

		if (!replaced){
			replaced = 1;
			*list = return_list;
		}
		return 0; /* drop */
	}

	/* accept parameters and add some values after it */
	if ((strcmp(name, "some_param3") == 0) &&
	    (strcmp(val,  "some_value3") == 0)) {
		// NULL terminated list of strings to add after "some_param3=some_value3"
		static const char *return_list[] = {"other_param3=other_value3", "other_param4=other_value4", NULL};
		// protection against double addition
		static int added = 0;

		if (!added){
			added = 1;
			*list = return_list;
		}
		return 1; /* accept */
	}

	return 1; /* accept by default */
}
#endif

static int __init proc_cmdline_init(void)
{
	modify_cmd_line(saved_command_line, modify_cmd_line_callback);
	proc_create("cmdline", 0, NULL, &cmdline_proc_fops);
	return 0;
}
fs_initcall(proc_cmdline_init);
