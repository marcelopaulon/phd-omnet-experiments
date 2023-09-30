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

network DADCA_ACK_n100_u4_a8000m2_1
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
        @display("i=misc/sensor;p=901.92,7865.20");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1562.80,7922.09");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2381.89,7863.19");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3854.76,7960.86");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4793.12,7791.60");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5338.04,7957.40");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=197.34,7565.19");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=524.36,7704.34");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1499.03,7507.00");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3266.07,7718.98");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5430.36,7609.32");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=500.06,7343.94");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1006.24,7444.72");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2008.11,7493.46");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2751.05,7371.76");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3584.79,7469.58");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4065.08,7403.66");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4829.05,7359.64");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5983.36,7497.88");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=120.11,7039.05");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1598.31,7011.24");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3170.15,7230.00");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3799.15,7099.94");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=954.13,6960.50");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1913.62,6817.60");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2249.90,6983.22");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2807.26,6959.58");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3278.27,6886.21");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5179.81,6959.60");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5533.16,6953.63");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5933.15,6891.55");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1142.22,6548.09");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1633.62,6590.75");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2638.48,6625.45");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4123.08,6606.30");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4973.59,6633.17");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=472.91,6443.78");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3651.66,6438.16");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4533.98,6324.66");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5381.19,6484.61");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5695.30,6276.85");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=82.64,6061.86");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=544.08,6084.79");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1426.29,6214.98");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2116.82,6228.72");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3027.37,6260.76");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4061.53,6165.63");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5341.56,6038.74");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1035.32,5795.52");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1889.46,5933.48");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2578.17,5926.46");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2944.78,5815.80");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3490.40,5979.84");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3957.21,5814.18");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4940.54,5984.30");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=584.32,5614.46");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1532.97,5662.70");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2253.71,5731.60");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4614.44,5709.77");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5219.49,5607.22");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=339.39,5339.83");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1159.30,5387.11");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2021.70,5425.55");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2637.43,5512.62");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3028.03,5338.27");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3397.34,5447.89");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4150.03,5400.88");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4770.06,5319.43");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5761.60,5471.19");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=684.23,5228.64");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2583.12,5156.11");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3809.18,5145.80");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5425.05,5086.05");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=68.84,4868.95");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1079.98,4993.28");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1475.37,5024.38");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2069.31,4997.47");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3105.41,4944.79");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3496.28,4984.23");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4271.14,4821.48");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5981.16,5028.18");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=609.70,4697.69");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2339.53,4658.38");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2676.09,4782.12");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3910.15,4672.63");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4879.02,4643.98");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5335.11,4542.88");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5937.16,4574.39");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=909.64,4451.58");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1912.07,4413.54");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3158.40,4465.11");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3543.79,4513.94");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4344.65,4445.42");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=239.41,4269.91");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=745.55,4083.63");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1239.37,4112.87");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2197.22,4092.13");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2693.98,4204.17");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4071.43,4159.09");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4708.26,4279.33");
    }

}

