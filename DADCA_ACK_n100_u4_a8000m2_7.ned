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

network DADCA_ACK_n100_u4_a8000m2_7
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
        @display("i=misc/sensor;p=426.28,7961.16");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2528.33,7959.28");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3371.84,7979.61");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3712.93,7849.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4097.22,7950.35");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5240.95,7882.93");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=147.46,7542.57");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=725.31,7692.19");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1316.77,7707.48");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2186.59,7528.56");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2555.65,7607.55");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3032.16,7664.57");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4745.51,7727.35");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5735.53,7674.12");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=772.38,7328.80");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1198.29,7265.26");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1803.96,7396.20");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2809.84,7349.55");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3785.40,7344.65");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4153.60,7360.57");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1916.62,7014.26");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2258.66,7127.75");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3441.54,7032.29");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4775.93,7140.69");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5362.42,7236.55");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=86.64,6858.78");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=601.35,6920.84");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1022.98,6898.06");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1643.62,6771.43");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2438.89,6807.52");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3064.65,6938.79");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4070.54,6828.85");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=796.55,6549.50");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1259.09,6583.63");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2767.02,6640.63");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3355.67,6605.73");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3763.19,6549.60");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4480.39,6664.86");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4895.30,6733.88");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5544.98,6712.50");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=262.43,6280.14");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1901.68,6433.11");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2434.04,6361.59");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4283.20,6290.17");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4645.67,6309.45");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5136.22,6475.38");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5819.63,6424.77");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=873.13,6105.86");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1493.79,6138.91");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3170.68,6229.00");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3703.03,6094.74");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4965.91,6076.63");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5394.45,6223.74");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=270.62,5857.80");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1993.92,5851.00");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2911.95,5902.20");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4298.34,5906.69");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5853.64,5865.76");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1427.78,5727.01");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2470.52,5538.87");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3424.21,5637.64");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3857.33,5719.73");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4746.29,5537.63");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=184.91,5387.29");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=709.02,5371.92");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1626.16,5354.82");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2013.41,5355.30");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2820.38,5440.54");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4340.40,5363.21");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5362.24,5519.64");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1149.37,5132.15");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1840.24,5045.31");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3474.99,5230.39");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4845.25,5147.08");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5958.11,5164.09");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=591.35,4823.51");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=952.95,4839.82");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1535.19,4806.58");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2255.94,4913.67");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2721.95,5003.96");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3163.98,5021.54");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3779.74,4803.65");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4255.67,4898.15");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5380.79,4832.85");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=217.43,4771.32");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2057.44,4561.19");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2815.74,4619.95");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3262.62,4653.42");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4754.65,4609.21");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=736.01,4500.23");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1315.66,4520.35");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3347.27,4313.54");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3847.28,4350.18");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5718.78,4528.69");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=362.32,4220.63");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1215.69,4106.01");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1750.03,4283.76");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2243.47,4253.82");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4440.60,4047.29");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5314.01,4165.17");
    }

}

