from supsisim.RCPblk import RCPblk
from numpy import size

def FH_5XXX_setTQBlk(pin, candev, ID):
    """

    Call:   FH_5XXX_setTQBlk(pin, ID)

    Parameters
    ----------
       pin: connected input port(s)
       ID : Device ID
  
    Returns
    -------
        blk: RCPblk

    """
    
    blk = RCPblk('FH_5XXX_setTQ', pin, [], [0,0], 1, [], [ID], candev)
    return blk

