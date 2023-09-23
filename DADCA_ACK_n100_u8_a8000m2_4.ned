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

network DADCA_ACK_n100_u8_a8000m2_4
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
        @display("i=misc/sensor;p=275.50,7761.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1003.84,7888.12");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1361.06,7893.01");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1891.25,7793.08");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2921.87,7810.57");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3518.59,7992.82");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3950.91,7872.14");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4594.49,7798.82");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4990.81,7811.72");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1543.00,7553.58");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2256.94,7696.80");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3464.44,7560.64");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5309.25,7619.30");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5812.76,7673.75");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=273.32,7379.91");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=722.70,7426.44");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1147.44,7350.53");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2706.17,7258.65");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4155.93,7430.44");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4737.54,7366.49");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1428.13,7115.40");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2076.26,7219.63");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3304.94,7169.83");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3903.00,7136.26");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5712.61,7239.91");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=401.22,6903.89");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=788.22,6772.21");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1811.05,6970.02");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2487.74,6888.29");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2935.27,6827.52");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3669.32,6850.35");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4288.05,6883.17");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4958.35,6836.87");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5288.41,6993.66");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5638.67,6774.35");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1175.32,6534.53");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1528.54,6762.40");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2060.21,6581.56");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3354.65,6602.96");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4041.44,6603.09");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4586.72,6642.34");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5248.78,6541.66");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=28.09,6380.40");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=680.63,6399.99");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2659.18,6478.67");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3219.66,6278.77");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4936.31,6276.12");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5921.29,6362.58");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=579.23,6062.17");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1032.93,6022.21");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2339.08,6247.89");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3528.90,6076.38");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4036.79,6126.10");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4389.07,6087.18");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5362.48,6108.06");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=16.26,5947.99");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1651.66,5893.77");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2006.74,5945.45");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2571.04,5790.24");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3157.92,5786.28");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3924.34,5774.22");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5853.77,5841.01");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=593.22,5671.93");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1075.50,5653.56");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3519.21,5651.25");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4483.82,5603.69");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5138.23,5719.38");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1443.23,5448.96");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1943.48,5440.71");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2686.61,5352.04");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3256.87,5394.05");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4155.28,5408.88");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5026.01,5293.34");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=215.28,5173.70");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=956.54,5273.07");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1556.61,5067.21");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2269.49,5265.62");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3761.12,5254.06");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5499.08,5259.16");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5844.75,5192.34");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=738.53,4922.67");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1983.49,5011.43");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3204.85,4870.63");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3724.32,4789.23");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4384.03,4999.92");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5085.86,4870.24");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=151.01,4638.64");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1348.28,4661.38");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2464.60,4760.69");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4132.81,4716.10");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4621.94,4637.89");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5491.82,4758.12");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5849.96,4614.92");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=496.69,4329.17");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1848.70,4428.66");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2847.41,4516.84");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3429.23,4474.90");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3877.33,4375.89");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5248.80,4449.65");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1158.38,4266.46");
    }

}

