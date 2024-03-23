from supsisim.RCPblk import RCPblk
from numpy import size

def FH_5XXX_getTQBlk(pout, candev, ID):
    """

    Call:   FH_5XXX_getTQBlk(pout, ID)

    Parameters
    ----------
       pout: connected output port(s)
       ID : Device ID

    Returns
    -------
       blk: RCPblk

    """

    blk = RCPblk('FH_5XXX_getTQ', [], pout, [0,0], 0, [], [ID], candev)
    return blk

