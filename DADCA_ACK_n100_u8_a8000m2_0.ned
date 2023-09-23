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

network DADCA_ACK_n100_u8_a8000m2_0
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
        @display("i=misc/sensor;p=2147.67,7927.63");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2563.29,7939.98");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3680.57,7786.92");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4775.48,7966.04");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=977.55,7590.31");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1741.43,7724.09");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2958.07,7677.07");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4194.39,7714.56");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4634.32,7517.65");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5187.31,7563.65");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=302.08,7437.36");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1296.92,7324.19");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2530.62,7494.66");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3262.31,7448.26");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3945.20,7413.41");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=63.98,7177.56");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1873.67,7166.77");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2985.21,7026.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4375.81,7085.19");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4807.08,7122.26");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5275.80,7215.94");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5744.81,7165.97");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=471.97,6794.76");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=950.68,6896.93");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1342.83,6831.62");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1787.11,6817.92");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2507.51,6997.28");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3494.63,6943.19");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3961.49,6802.10");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4965.23,6783.18");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5459.48,6867.78");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5907.40,6854.93");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=126.49,6669.42");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=788.91,6580.28");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2730.32,6673.61");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4560.64,6741.51");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1130.32,6394.06");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1562.83,6489.08");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2098.77,6496.02");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2679.70,6322.41");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3346.00,6499.47");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3944.23,6307.89");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4266.13,6511.21");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4819.42,6425.04");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5175.15,6436.60");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5796.28,6384.61");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=213.34,6199.60");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=763.07,6222.72");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3075.79,6071.32");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3451.48,6121.49");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4443.90,6064.68");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5080.39,6079.03");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=901.38,5860.95");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1314.83,5836.12");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1795.97,5993.51");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2207.34,5886.15");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2852.22,5791.64");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3698.74,5806.65");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4074.24,5913.61");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5483.66,5827.13");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5858.95,5921.33");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=449.65,5707.57");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1749.93,5606.51");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2403.82,5553.38");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3368.42,5682.99");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4360.81,5525.70");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5043.66,5704.93");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=757.45,5386.34");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2013.67,5309.36");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3716.96,5299.21");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5986.35,5306.22");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1112.98,5251.15");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1665.59,5253.93");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2585.85,5134.07");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2969.50,5245.89");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4782.75,5240.81");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1294.55,4947.06");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1639.03,4794.41");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1992.79,4888.25");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4039.81,4910.78");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4805.38,4889.32");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5260.45,4888.86");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5781.77,4833.76");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=962.55,4668.64");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2231.77,4631.89");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2603.92,4776.53");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3182.77,4758.07");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=20.50,4341.99");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=523.13,4388.88");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1389.22,4439.35");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2656.10,4420.96");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3785.74,4339.01");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4472.55,4531.55");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5709.75,4320.41");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=984.26,4199.08");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1832.67,4143.24");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3433.72,4144.87");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4216.24,4274.87");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4851.45,4240.60");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=94.62,3937.17");
    }

}

