typedef struct __timer0_thread_struct {
	// "persistent" data for this "lthread" would go here
	//int	data;
        unsigned int	msgcount;
} timer0_thread_struct;

void init_timer0_lthread(timer0_thread_struct *);
int timer0_lthread(timer0_thread_struct *,int,int,unsigned char*);
