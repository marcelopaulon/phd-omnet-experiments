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

network DADCA_ACK_n100_u16_a8000m2_0
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
        @display("i=misc/sensor;p=869.17,7829.17");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1659.92,7815.81");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2550.12,7787.38");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3574.05,7909.69");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4563.13,7898.01");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5173.26,7911.05");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5691.39,7953.46");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=261.42,7745.42");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=536.39,7514.98");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2960.90,7665.56");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4096.23,7623.63");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5574.64,7556.42");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1025.87,7293.06");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1472.97,7356.41");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1856.58,7457.49");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3566.91,7332.45");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4663.86,7382.40");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5029.32,7401.64");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5983.18,7399.50");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=52.06,7247.61");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1998.82,7061.04");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2458.57,7248.95");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3197.66,7156.41");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5732.75,7103.26");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=558.64,6960.78");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1136.40,6959.84");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2935.30,6922.16");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3770.74,7005.83");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4129.85,6897.76");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5293.94,6927.42");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=112.21,6722.63");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=473.29,6579.17");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1805.32,6557.28");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2423.04,6631.71");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2884.10,6516.90");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3629.52,6617.32");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4461.23,6758.47");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4909.68,6630.02");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5680.26,6520.48");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1241.21,6313.73");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3935.91,6290.28");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5398.59,6277.89");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=102.25,6078.02");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=593.98,6114.40");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2471.08,6078.37");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3351.14,6215.49");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4441.83,6235.81");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4825.29,6121.91");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5989.96,6089.98");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1108.56,5830.83");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2017.69,5956.67");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3025.25,5910.14");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3877.93,5816.45");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4236.09,5816.44");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4991.33,5812.35");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5390.98,5839.86");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=655.77,5670.38");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1770.62,5594.55");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2169.44,5637.81");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3365.85,5537.34");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5834.67,5700.14");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=32.70,5506.43");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=382.27,5300.09");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1174.36,5369.36");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2885.68,5479.73");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3779.79,5477.53");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4801.06,5507.18");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5203.46,5441.46");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=754.22,5258.97");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1680.77,5085.29");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2082.86,5112.60");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2595.11,5084.57");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3229.95,5164.86");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4265.06,5122.81");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5863.53,5139.15");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=527.49,4934.98");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1203.89,4901.85");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3653.55,4922.40");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4535.01,4898.43");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4917.67,4878.25");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5439.50,4877.25");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=307.09,4637.76");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1966.42,4551.51");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2623.84,4677.07");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3417.43,4590.75");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=57.09,4371.54");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1068.29,4513.35");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1428.63,4448.25");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2465.52,4357.63");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2992.76,4499.41");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3697.03,4332.62");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4342.33,4479.84");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4801.04,4454.23");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5594.21,4303.44");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=595.22,4182.86");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1143.99,4111.70");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1921.30,4188.81");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3366.81,4198.50");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4020.42,4150.47");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4645.49,4075.18");
    }

}

