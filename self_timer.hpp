#ifndef SelfTimer_HPP
#define SelfTimer_HPP


class SelfTimer
{
public:
    /* Constructor */
    SelfTimer();
    /* DeConstructor */
    ~SelfTimer();

    void create(int delta_t);
    void start(void);

    int is_over(void);
    void reset(void);

private:
    static void elapsed(void);

};


#endif