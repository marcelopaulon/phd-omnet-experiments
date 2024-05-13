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

network DADCA_ACK_n100_u8_a8000m2_8
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
        @display("i=misc/sensor;p=173.81,7897.07");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1829.32,7997.52");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2222.07,7786.32");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3251.78,7784.83");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3813.48,7936.41");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4202.50,7964.79");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5445.39,7952.71");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5746.32,7766.09");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1470.26,7736.13");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2773.30,7723.53");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4172.00,7586.19");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5090.58,7731.67");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=321.13,7469.48");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=965.41,7385.60");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2506.56,7352.48");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3080.65,7336.21");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3668.08,7429.47");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4674.91,7504.63");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5596.02,7420.61");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1307.72,7194.15");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1764.21,7209.67");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3448.75,7040.31");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4393.33,7251.30");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4810.38,7179.23");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=481.42,6957.08");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=870.56,6904.48");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2001.41,6934.15");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2454.10,6861.23");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3058.40,6978.56");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3966.98,6940.21");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5340.66,6989.61");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5873.55,6982.70");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1414.05,6725.50");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2922.29,6588.84");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3796.13,6516.46");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4518.48,6726.13");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5100.66,6625.49");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=510.35,6510.27");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=998.94,6267.93");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1527.43,6363.47");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1899.72,6452.10");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2559.76,6482.20");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3250.36,6442.23");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4151.97,6389.70");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4505.63,6374.37");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5389.99,6394.98");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1288.56,6047.09");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2160.10,6183.68");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2822.16,6164.50");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3660.50,6025.32");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5055.34,6145.86");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5854.85,6106.68");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=131.39,5806.39");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=484.57,5914.65");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=934.54,5918.55");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1768.97,5933.37");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3098.13,5914.27");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4281.04,5883.74");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4876.03,5822.94");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5259.40,5856.52");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2452.30,5690.43");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2811.08,5707.73");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3509.84,5537.04");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5463.68,5571.85");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=206.00,5319.60");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=964.40,5476.54");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1663.13,5476.58");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2025.50,5474.12");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3018.12,5394.27");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3951.62,5371.94");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4464.74,5280.02");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4937.06,5457.31");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=639.74,5222.29");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1491.47,5124.65");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2139.11,5075.84");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3486.16,5145.10");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=434.20,4795.72");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=976.24,4942.67");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2895.73,4995.34");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4051.97,4868.58");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4729.72,4903.26");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5147.57,4941.51");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5514.93,4966.23");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1977.65,4742.27");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2359.07,4660.82");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5101.15,4547.07");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5474.74,4572.98");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=66.57,4524.95");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=561.23,4406.22");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=900.02,4534.26");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1416.97,4476.46");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2825.24,4318.14");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3193.65,4457.92");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3600.56,4485.00");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4140.35,4397.44");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5829.67,4511.58");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1058.34,4128.50");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1756.07,4084.63");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2410.95,4238.52");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3593.05,4100.21");
    }

}

