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

network DADCA_ACK_n100_u3_a8000m2_0
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
        @display("i=misc/sensor;p=482.58,7777.37");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1280.29,7940.66");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1972.75,7954.94");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2445.31,7764.04");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2790.99,7960.54");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3748.21,7945.15");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4364.26,7895.14");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=132.14,7591.11");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=845.82,7639.29");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1446.61,7628.98");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3298.81,7703.83");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4032.51,7732.84");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4727.03,7724.72");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1467.18,7276.06");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1940.43,7490.63");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2923.27,7460.72");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3581.79,7480.85");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4285.28,7464.05");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5117.75,7468.94");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5719.29,7269.64");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=422.64,7249.33");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2151.96,7142.92");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2514.89,7205.94");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3481.57,7118.91");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4016.28,7086.44");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4735.72,7186.72");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3.08,6956.35");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=324.54,6809.88");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=733.61,6883.85");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1366.36,6917.93");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3127.80,6815.78");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3775.60,6832.16");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4360.28,6995.60");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5324.59,6833.98");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1050.59,6519.36");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1905.16,6575.41");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2360.09,6583.46");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4268.11,6592.44");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4961.21,6669.17");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5887.62,6661.69");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=173.92,6411.60");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=633.19,6495.65");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1336.51,6282.10");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2835.84,6450.62");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3504.15,6362.24");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3897.79,6350.58");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5333.57,6288.82");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=317.34,6041.90");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1018.99,6104.67");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2193.11,6192.89");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3138.84,6212.14");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4518.31,6244.15");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4897.09,6142.68");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2519.10,5983.21");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3888.51,5956.25");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4349.87,5926.48");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5753.08,5908.85");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=901.15,5764.98");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1363.77,5654.46");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1760.86,5729.13");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3096.92,5667.88");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3613.24,5558.51");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5045.88,5673.89");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=513.71,5414.31");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=889.23,5401.66");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2020.26,5413.55");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3243.90,5326.10");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4457.87,5376.35");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4901.96,5291.98");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5507.88,5383.81");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2568.96,5255.95");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3910.56,5129.92");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5211.26,5120.88");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=306.19,4969.44");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=750.68,4898.64");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1255.19,4974.20");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1673.94,4957.76");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2146.11,4937.86");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3034.61,4894.59");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3652.10,4864.99");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4212.46,4920.85");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=7.05,4716.34");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3342.14,4584.29");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4823.55,4692.35");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5444.58,4722.63");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=599.70,4468.81");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=976.83,4501.35");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1641.73,4518.32");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2039.72,4392.09");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2576.90,4415.89");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3705.06,4358.95");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4172.16,4483.63");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5189.29,4292.51");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5844.10,4382.58");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=730.71,4133.12");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1228.30,4127.14");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3212.77,4251.31");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=352.52,3822.18");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1715.76,3942.18");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2708.91,3985.86");
    }

}

