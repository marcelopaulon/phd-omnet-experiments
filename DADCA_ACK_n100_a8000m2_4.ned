package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n100_a8000m2_4
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=208.77,7987.39");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=931.70,7960.46");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1978.91,7929.39");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3104.54,7927.22");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3586.72,7766.85");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4483.65,7870.88");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=753.80,7501.58");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1220.42,7727.26");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4234.14,7596.53");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4903.33,7509.63");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1036.66,7266.17");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1873.70,7266.51");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2625.23,7229.51");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3127.88,7459.66");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4455.47,7290.09");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5755.42,7382.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=386.27,7076.82");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2176.20,7011.46");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3203.89,7039.20");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3904.44,7176.05");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1537.60,6958.64");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3558.28,6866.96");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4572.61,6879.53");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5150.71,6946.29");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1022.65,6628.70");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1453.87,6450.97");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1829.38,6534.37");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2518.09,6577.10");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3077.29,6557.80");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3599.00,6475.99");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5476.06,6702.48");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5909.73,6677.29");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=121.99,6369.18");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=499.30,6408.91");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1137.79,6264.90");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3222.57,6217.40");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4268.80,6202.48");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4768.87,6440.91");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1657.35,5962.42");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2484.71,6096.30");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2886.56,5972.89");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5283.79,6113.36");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5848.12,5956.20");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=284.60,5742.79");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=808.98,5783.39");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1187.85,5802.13");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3642.34,5772.90");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4249.29,5769.13");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4872.39,5742.92");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5426.70,5729.96");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=517.14,5433.67");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2007.70,5621.53");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2639.63,5604.74");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3238.62,5480.90");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4645.38,5414.32");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=209.14,5201.37");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=939.09,5194.56");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1354.50,5301.86");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3498.26,5163.53");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4026.02,5302.91");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5319.80,5327.00");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2142.82,4921.50");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2573.36,4907.54");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4662.12,4957.23");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5326.69,4960.46");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5897.79,5147.11");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=109.72,4768.82");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2934.55,4840.29");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3553.30,4715.24");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4020.24,4707.37");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4975.50,4677.24");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5734.03,4797.21");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=575.72,4596.39");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1297.62,4571.52");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2843.26,4401.50");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3201.21,4556.47");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4402.72,4623.65");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5671.88,4385.23");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=10.77,4295.97");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=818.51,4297.49");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4050.43,4237.08");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=318.63,3919.17");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1779.92,4067.25");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3708.19,4037.35");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4134.60,3863.11");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4528.31,4051.81");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=675.54,3715.12");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1224.40,3830.09");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2686.87,3824.94");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3218.36,3832.44");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4747.71,3611.80");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5171.50,3750.20");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5721.69,3822.28");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=844.64,3376.51");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1234.47,3402.53");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1599.10,3465.43");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2113.55,3556.11");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2718.94,3404.80");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3937.08,3405.88");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5242.61,3362.79");
    }

}

