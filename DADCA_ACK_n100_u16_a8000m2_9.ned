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

network DADCA_ACK_n100_u16_a8000m2_9
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
        @display("i=misc/sensor;p=118.77,7754.71");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1772.80,7923.65");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2657.37,7897.46");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3431.07,7940.17");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5815.39,7929.94");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=660.20,7745.83");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1102.46,7723.08");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1686.87,7512.41");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2139.38,7646.24");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3058.36,7667.95");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3414.77,7575.54");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4002.72,7563.68");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4627.56,7720.72");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5250.38,7619.77");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=77.34,7337.40");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=531.52,7310.46");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=881.59,7349.34");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1979.01,7312.78");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5674.01,7413.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1145.41,7084.88");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2383.12,7108.61");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2865.89,7207.81");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3243.46,7127.82");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3928.37,7109.92");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4604.53,7238.88");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5232.35,7015.31");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=140.66,6945.69");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=818.78,6779.45");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1692.65,6958.40");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2575.45,6769.80");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4881.90,6856.17");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5725.76,6892.85");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=361.09,6600.75");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1710.36,6525.04");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2173.68,6751.61");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3131.36,6660.36");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3960.51,6614.06");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4520.84,6745.94");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=89.64,6289.56");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=849.94,6309.49");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1375.63,6356.41");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2111.48,6369.41");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2695.02,6415.15");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3574.36,6359.26");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4524.13,6339.94");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4918.62,6324.05");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5328.09,6464.42");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5760.39,6478.35");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=487.80,6126.14");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3159.36,6252.20");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4112.96,6198.28");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5839.26,6081.33");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1068.03,5915.41");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1604.30,5869.19");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2000.65,5991.77");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2467.49,6005.32");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3083.89,5863.60");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3840.83,5976.50");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4595.81,5975.39");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5385.11,5944.78");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=238.46,5625.14");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=587.10,5563.84");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2749.87,5730.18");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3367.15,5635.82");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5672.50,5740.59");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1531.71,5453.30");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1946.77,5485.41");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2357.13,5391.61");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3938.09,5484.77");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4447.88,5498.80");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=451.96,5108.63");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1025.64,5118.81");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1575.15,5088.28");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2734.44,5202.23");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3125.92,5237.26");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4683.73,5190.02");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5299.09,5190.61");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=6.10,4928.19");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2012.69,4849.74");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3066.16,4863.47");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3530.82,5029.17");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4013.07,5021.37");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4726.94,4803.47");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=708.31,4554.08");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1146.78,4660.58");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1564.84,4550.88");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2419.36,4778.91");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3514.73,4656.10");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5371.39,4671.92");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5932.01,4591.47");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=150.52,4492.66");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4304.26,4419.41");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4711.27,4355.27");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5045.49,4510.09");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=341.85,4056.18");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1019.37,4128.06");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1914.75,4191.07");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2291.16,4284.30");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2734.27,4282.92");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3245.91,4084.57");
    }

}

