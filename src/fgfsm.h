// (c) Francois Guibert, www.frozax.com (@Frozax)
#pragma once

template<typename T>
class fgFSM
{
public:
    fgFSM() : _time_in_cur_state(0.0f), _cur_state(-1)
    {
    }
    
    virtual void BeginState( T state ) {}
    virtual void UpdateState( T state ) {}
    virtual void EndState( T state ) {}
    
    void SetState( T state )
    {
        EndState( (T)_cur_state );
        _cur_state = state;
        _time_in_cur_state = 0.0f;
        BeginState( (T)_cur_state );
    }
    
    void UpdateFSM( float delta_time )
    {
        if( _cur_state != -1 )
        {
            _time_in_cur_state+=delta_time;
            UpdateState( (T)_cur_state );
        }
    }
    
    float GetTimeInCurState() { return _time_in_cur_state; }
    T GetState() { return (T)_cur_state; }
    
private:
    float _time_in_cur_state;
    int _cur_state;
};