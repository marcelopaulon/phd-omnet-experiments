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

network DADCA_ACK_n100_u4_a8000m2_8
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
        @display("i=misc/sensor;p=234.28,7959.79");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1240.92,7799.12");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1659.80,7911.78");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2228.37,7814.67");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4764.93,7795.62");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5806.67,7784.61");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3338.58,7667.52");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3735.66,7570.50");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4084.92,7653.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=180.44,7494.54");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=726.30,7481.32");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1085.59,7473.65");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1650.53,7419.55");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2695.06,7479.71");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3280.93,7279.94");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4669.79,7390.06");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5489.78,7458.72");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=462.92,7243.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1317.98,7191.44");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2189.07,7253.51");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2753.57,7121.84");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3639.71,7190.82");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4123.49,7034.44");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1590.62,6925.64");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2426.05,6907.91");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3124.08,6939.62");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4987.09,6878.88");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5638.25,6880.88");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=95.09,6649.35");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1044.34,6691.86");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2111.88,6671.52");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2525.67,6545.89");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3781.93,6652.59");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4684.76,6688.35");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5393.08,6625.45");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=170.81,6295.52");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=642.69,6440.92");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2821.87,6352.83");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3174.49,6491.07");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5717.91,6327.61");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1208.33,6158.16");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1844.65,6113.31");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2379.39,6195.16");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3180.82,6073.54");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3871.27,6091.12");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5060.23,6230.74");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=259.86,5860.23");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=703.64,5953.09");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2493.84,5839.65");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3527.89,6006.52");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4696.88,6011.73");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5205.84,5808.58");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5603.93,5963.31");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1101.24,5659.55");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1455.60,5546.48");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1986.40,5758.45");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4030.60,5624.12");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4504.35,5576.24");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4891.87,5567.05");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=493.83,5492.85");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=869.86,5392.14");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2145.00,5319.28");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2581.82,5293.32");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3008.42,5495.59");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5183.24,5341.53");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5485.34,5520.83");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4.51,5214.30");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1180.40,5205.70");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3443.20,5158.76");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4792.76,5215.37");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5609.19,5142.25");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=307.82,4836.58");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=737.12,4944.96");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1725.53,4846.91");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2872.73,4918.10");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4059.11,5015.91");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4478.55,5012.18");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4901.69,4881.45");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1407.09,4617.25");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2366.76,4571.14");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3178.33,4563.69");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3538.51,4712.48");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4688.59,4574.71");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5372.32,4670.82");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=832.67,4529.95");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2003.13,4391.96");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2733.71,4516.81");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4111.94,4466.79");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5716.64,4474.24");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=957.13,4100.21");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1714.40,4185.55");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2948.69,4066.81");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3745.89,4286.13");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4529.85,4233.19");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4946.15,4190.07");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5702.62,4111.64");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=59.27,4024.77");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=500.26,4012.12");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1372.41,3847.65");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2416.03,4020.98");
    }

}

