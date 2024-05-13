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

network DADCA_ACK_n100_u2_a8000m2_2
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
        @display("i=misc/sensor;p=3922.48,7890.85");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4482.82,7762.77");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1310.61,7724.12");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1706.74,7538.96");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2925.27,7531.67");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3277.14,7720.45");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=584.97,7415.46");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3855.50,7428.70");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4224.05,7414.56");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=58.82,7121.36");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1143.75,7166.14");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1760.33,7155.66");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2130.38,7160.11");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2786.93,7110.78");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3176.25,7016.14");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3474.20,7229.54");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4728.91,7256.38");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=349.85,6814.73");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=802.64,6935.47");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1184.63,6795.98");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1659.25,6778.55");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2345.43,6867.87");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3818.02,6899.07");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4315.83,6800.34");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4744.64,6887.75");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5228.29,6813.18");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5692.06,6960.28");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2027.01,6656.16");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2593.06,6568.69");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2970.91,6703.55");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3340.83,6643.45");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=906.77,6505.80");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1381.78,6333.51");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3040.27,6278.24");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3638.27,6429.38");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5156.08,6393.06");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5722.47,6355.42");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=554.35,6177.10");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1996.05,6241.12");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2508.15,6193.89");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4308.54,6149.12");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4805.24,6249.29");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1617.09,6002.16");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3416.79,5908.72");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5062.60,5857.81");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5524.47,5941.89");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=555.33,5728.61");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1058.74,5695.49");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1896.00,5652.49");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2260.75,5610.72");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3032.97,5537.54");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4558.39,5584.78");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5372.37,5527.66");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5822.87,5691.44");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=286.20,5399.52");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1561.70,5478.82");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2011.85,5287.11");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3915.17,5514.46");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4875.23,5283.39");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1184.01,5175.96");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1585.30,5052.98");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2613.65,5107.50");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3279.65,5096.10");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3629.29,5274.31");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4241.43,5148.86");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=386.93,4983.25");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2225.60,4931.84");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2915.71,4839.48");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3631.11,4913.26");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4057.71,4840.82");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4741.63,4810.48");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5260.34,4863.08");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5774.75,5023.60");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=922.49,4611.97");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1322.13,4555.11");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1652.63,4684.66");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3272.88,4667.56");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3642.88,4551.04");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4492.66,4557.83");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=134.63,4421.59");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2447.33,4458.72");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4151.32,4308.97");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4737.22,4297.39");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5806.87,4507.57");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=529.73,4151.23");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1025.48,4206.70");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1703.79,4217.58");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2281.59,4105.26");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3075.95,4241.56");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5076.51,4095.67");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=225.94,3846.77");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2596.44,3853.17");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4579.81,3922.70");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=718.29,3649.90");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1199.49,3639.10");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1660.32,3586.64");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2106.71,3614.15");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3057.14,3720.80");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3931.39,3708.28");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5197.52,3603.10");
    }

}

