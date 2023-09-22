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

network DADCA_ACK_n100_a8000m2_2
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
        @display("i=misc/sensor;p=282.90,7789.06");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1703.42,7917.74");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2774.58,7895.74");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4944.79,7788.17");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=770.84,7620.15");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2391.78,7683.73");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3174.29,7723.59");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3694.68,7537.36");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4214.67,7722.28");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5614.22,7604.16");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1486.06,7452.34");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1927.27,7385.83");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2729.09,7460.43");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4459.18,7374.62");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=847.83,7188.57");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2499.11,7091.84");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3266.91,7205.66");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4789.11,7151.03");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5425.46,7177.12");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=379.91,6802.28");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=888.61,6782.04");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1593.17,6755.42");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2870.52,6756.51");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4149.78,6963.06");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4944.37,6747.67");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5893.21,6854.96");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1219.94,6603.38");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2250.72,6507.70");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3248.13,6595.91");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3601.77,6478.97");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4514.95,6483.69");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=677.40,6331.29");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1851.11,6385.17");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2611.83,6425.96");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4057.34,6266.92");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5409.47,6331.82");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=24.41,5990.11");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1108.87,6106.56");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1571.99,5951.34");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2142.16,6089.21");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2765.59,6006.18");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3261.61,6122.76");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5101.68,6114.80");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=449.37,5855.00");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1224.64,5692.58");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1903.71,5772.07");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2473.77,5695.91");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3580.37,5883.22");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4054.34,5852.52");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4533.72,5893.70");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5478.85,5693.99");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=809.86,5596.11");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2995.43,5573.24");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3517.48,5458.95");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4462.63,5439.72");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5061.69,5527.97");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=127.63,5362.90");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1155.33,5186.38");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1578.92,5373.27");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3943.26,5378.72");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5590.36,5177.52");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5974.51,5218.33");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=338.54,4986.19");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=761.69,5123.79");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2190.98,5089.13");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2636.26,5073.92");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4035.50,4978.08");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=975.14,4679.97");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1420.76,4867.50");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2001.86,4684.70");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3159.79,4820.80");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4345.94,4778.25");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5023.00,4834.80");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=571.25,4624.11");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1528.10,4465.43");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2708.62,4594.42");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3844.75,4459.66");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4562.47,4413.09");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5608.35,4507.57");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=507.80,4255.09");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1136.17,4242.59");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2430.01,4143.17");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2903.30,4231.52");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3328.56,4286.69");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4221.01,4265.46");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5206.23,4222.45");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=773.90,3942.41");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1748.78,4070.03");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3813.13,3941.69");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4718.72,4061.90");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5686.58,4009.61");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=438.33,3690.92");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1177.16,3781.89");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2952.23,3853.00");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4898.46,3624.73");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5394.98,3620.14");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5898.53,3644.35");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=152.32,3371.53");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1754.15,3370.67");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2504.85,3581.58");
    }

}

