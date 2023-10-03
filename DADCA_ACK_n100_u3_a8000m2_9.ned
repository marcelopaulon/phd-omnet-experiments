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

network DADCA_ACK_n100_u3_a8000m2_9
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
        @display("i=misc/sensor;p=1545.15,7783.73");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=764.15,7719.84");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1133.28,7633.21");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2209.13,7526.96");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2803.78,7693.29");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4208.36,7642.57");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4678.64,7569.03");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5224.52,7691.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5834.07,7741.68");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=28.10,7489.78");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=636.17,7333.96");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1234.02,7268.52");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3228.84,7452.57");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3578.35,7496.68");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=360.54,7074.62");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=866.64,7066.69");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1651.72,7218.81");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2059.19,7113.15");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2599.95,7205.61");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3708.67,7068.48");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4121.95,7200.15");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4923.22,7227.95");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5708.80,7188.66");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1439.66,6791.04");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2868.01,6788.71");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4499.79,6886.10");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4929.19,6862.52");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5291.92,6781.75");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=861.07,6600.61");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1888.91,6760.68");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2252.27,6698.78");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3218.83,6743.15");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3872.93,6660.60");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5928.26,6530.75");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=294.19,6460.52");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1323.24,6327.94");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2676.47,6389.67");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3070.42,6312.27");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4772.04,6490.00");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=868.78,6064.16");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1861.82,6257.71");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2335.11,6116.58");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3664.43,6121.84");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4143.19,6217.52");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4988.67,6206.75");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5350.90,6144.10");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=61.20,5885.41");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1358.50,5789.98");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1759.50,5815.86");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2686.02,5780.66");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3106.55,5945.75");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3492.51,5778.06");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4574.91,5885.06");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=809.14,5708.29");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2017.85,5574.16");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3975.04,5643.06");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5186.87,5677.34");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5678.10,5717.15");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=452.58,5336.51");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1610.61,5390.86");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2375.38,5391.78");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3159.10,5303.54");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4855.93,5457.38");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5531.37,5380.08");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=101.08,5256.85");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=689.54,5051.37");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1069.68,5238.36");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3765.49,5106.86");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4122.35,5067.06");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4520.03,5212.64");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5135.90,5173.69");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=259.57,4784.06");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1889.99,4964.20");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2512.27,4979.75");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3389.55,4864.33");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5473.37,4873.89");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=932.71,4745.23");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1547.70,4596.89");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2819.75,4580.08");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3738.19,4594.40");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4081.36,4688.55");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4725.83,4775.71");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5048.86,4618.13");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5838.77,4697.73");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1897.30,4289.66");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2426.49,4368.55");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3986.37,4313.31");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4762.57,4313.11");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5548.04,4398.88");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=217.52,4168.96");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=789.17,4086.46");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3076.13,4221.39");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3533.50,4143.97");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4420.89,4056.88");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5259.30,4159.23");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=12.19,3853.13");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1239.89,4024.87");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1617.72,4029.50");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2482.46,3945.86");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2912.35,3803.00");
    }

}

