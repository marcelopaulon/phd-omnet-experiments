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

network DADCA_ACK_n100_u4_a8000m2_3
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
        @display("i=misc/sensor;p=307.17,7892.52");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1172.81,7830.60");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1929.00,7839.43");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2482.26,7995.73");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2994.94,7757.03");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3432.13,7994.20");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3861.51,7770.18");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4641.69,7818.59");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5325.46,7997.24");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5718.29,7832.22");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1545.40,7726.79");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2417.11,7584.51");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=104.15,7403.68");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=726.46,7489.12");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1185.09,7417.22");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1694.74,7405.55");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3150.61,7339.22");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3544.13,7390.63");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4136.74,7449.56");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5027.24,7483.89");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5540.20,7391.86");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1634.94,7054.01");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3896.73,7143.75");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4419.71,7196.00");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5557.47,7032.48");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=378.22,6765.50");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=933.15,6916.57");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2325.60,6928.22");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2752.23,6992.10");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3130.91,6855.54");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4728.68,6804.49");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5110.49,6936.56");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2577.80,6592.75");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3731.09,6755.99");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4261.49,6545.44");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5279.20,6575.76");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=374.47,6358.73");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=826.34,6305.97");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1834.54,6504.05");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2352.50,6316.09");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3003.32,6289.24");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4679.06,6427.09");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5905.24,6514.98");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1376.87,6243.45");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1919.09,6124.28");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3564.94,6076.57");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4542.93,6028.86");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=380.15,5859.50");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=741.17,5943.75");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3026.74,5939.10");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5104.72,5954.68");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5551.40,5972.83");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1243.83,5606.64");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1676.81,5741.89");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2175.32,5713.22");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2740.52,5601.07");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3141.58,5529.34");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3520.14,5610.49");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4135.70,5770.89");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5087.56,5604.26");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5500.20,5582.18");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=578.97,5515.11");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2358.02,5297.68");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4672.28,5462.40");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=265.25,5121.20");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=835.09,5248.70");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1852.19,5134.30");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3069.67,5121.63");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4971.13,5042.70");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5474.62,5116.34");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1456.95,4948.00");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2670.38,4931.84");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3526.71,5008.33");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4175.65,5017.74");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4507.69,4884.32");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=889.89,4725.90");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2919.20,4641.11");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3837.81,4641.37");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5072.46,4690.94");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5524.95,4747.49");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=572.12,4389.06");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1216.47,4315.64");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1623.82,4443.07");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2181.79,4510.36");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3382.91,4446.45");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4312.61,4370.47");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4850.86,4359.32");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=8.76,4053.99");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2807.02,4043.79");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3239.19,4081.41");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3783.30,4114.97");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5383.22,4236.80");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5843.41,4283.30");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=896.87,3968.47");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1290.69,3859.43");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1786.12,4030.32");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2415.98,3952.03");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3519.26,3849.19");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4158.32,4009.54");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5850.81,3831.73");
    }

}

