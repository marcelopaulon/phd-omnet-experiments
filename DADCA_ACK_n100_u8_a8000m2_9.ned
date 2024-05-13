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

network DADCA_ACK_n100_u8_a8000m2_9
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
        @display("i=misc/sensor;p=1366.14,7760.79");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1774.29,7852.85");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3846.88,7777.16");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5215.93,7913.43");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=34.71,7590.50");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2346.19,7659.38");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2814.09,7526.50");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3292.26,7688.17");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4467.93,7581.68");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5065.42,7553.71");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5774.24,7610.84");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=419.10,7455.96");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1044.85,7342.69");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1948.72,7497.26");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3872.32,7419.83");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4815.15,7302.44");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=447.06,7065.78");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1400.90,7027.56");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1926.80,7128.37");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2392.12,7256.97");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2943.60,7195.32");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3383.26,7054.06");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4188.48,7202.59");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5413.12,7089.44");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=146.19,6808.64");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2481.91,6877.09");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4589.59,6921.68");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4955.29,6929.40");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5809.56,6997.13");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=566.72,6668.82");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=945.09,6595.66");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1310.39,6664.16");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3293.79,6652.21");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3868.48,6733.11");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5125.87,6518.39");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5473.57,6733.50");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1922.12,6386.36");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2829.65,6297.39");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3655.73,6333.00");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4110.17,6388.53");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4562.69,6504.35");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5525.62,6340.55");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=143.37,6245.67");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=805.53,6167.67");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1216.18,6157.89");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1765.68,6031.24");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2331.84,6228.98");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3406.73,6067.16");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4646.01,6032.39");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=427.28,5891.65");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1259.35,5774.82");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2070.28,5852.36");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2541.42,5780.38");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3027.02,5884.77");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4156.47,5844.30");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4969.62,5855.59");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5504.01,5976.26");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=958.41,5550.67");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1659.95,5670.05");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5625.07,5545.00");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4.20,5404.18");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2671.79,5403.51");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3089.62,5521.08");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3550.52,5512.64");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4545.97,5453.24");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5033.96,5499.99");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=306.49,5181.58");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=676.25,5252.53");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1089.28,5148.62");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1504.52,5237.97");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2005.25,5256.70");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2381.51,5110.41");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3392.49,5163.10");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4231.40,5203.29");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=9.30,4852.91");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1573.23,4880.72");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2799.07,4900.06");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3165.52,4806.26");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3923.37,4829.81");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4355.42,4842.67");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4711.46,4940.00");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5630.57,4963.18");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5979.89,4800.94");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=556.99,4669.37");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1213.46,4576.31");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2118.06,4782.34");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2476.91,4648.76");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5205.97,4655.97");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=147.12,4473.35");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1673.91,4443.54");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2945.58,4317.51");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3745.51,4303.39");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4265.94,4368.64");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4636.71,4365.33");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5929.26,4372.86");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=425.72,4112.38");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=803.83,4118.82");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1473.49,4047.22");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2139.36,4061.48");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2569.48,4284.46");
    }

}

