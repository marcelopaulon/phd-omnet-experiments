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

network DADCA_ACK_n100_u4_a8000m2_0
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
        @display("i=misc/sensor;p=462.51,7947.07");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1247.91,7849.34");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1629.69,7926.25");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2000.58,7937.60");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3247.67,7988.20");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3632.39,7779.20");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4269.29,7961.49");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4649.99,7917.34");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5184.71,7997.72");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=241.76,7629.30");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=912.61,7671.73");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2302.66,7554.22");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2902.27,7739.31");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3256.05,7596.64");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4285.76,7552.47");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4686.14,7533.41");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5155.37,7510.82");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5806.61,7739.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1299.48,7370.08");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1848.87,7274.62");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3964.33,7331.68");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=353.19,7249.94");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=863.12,7212.95");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2223.63,7156.22");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2999.42,7163.81");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3615.23,7134.57");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4623.13,7025.68");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5656.05,7158.65");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=301.84,6823.50");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=878.37,6862.36");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2607.49,6936.93");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3628.44,6774.65");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5257.69,6956.36");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1607.64,6620.07");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2274.71,6744.61");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2693.12,6575.03");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3184.37,6554.29");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4015.56,6730.78");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4592.16,6622.23");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5361.93,6561.52");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5908.34,6629.22");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=209.75,6412.56");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1161.89,6281.49");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2145.57,6349.42");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3832.42,6384.06");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4937.83,6486.55");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=771.76,6221.91");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1557.23,6080.93");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3028.54,6161.96");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5822.43,6202.90");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=221.48,5855.61");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1918.24,5825.68");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2499.83,5992.46");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3337.02,5990.13");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3992.32,6003.39");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4502.19,6003.73");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5069.93,5917.89");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5982.69,5844.64");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1384.57,5703.07");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2274.24,5547.88");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3221.17,5634.55");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3745.84,5753.07");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4211.40,5690.48");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5663.21,5699.78");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=12.83,5518.03");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=386.21,5355.78");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=812.87,5524.12");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1158.74,5366.65");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1772.14,5306.98");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2774.54,5510.00");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4639.13,5355.79");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5087.40,5414.14");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5530.93,5295.89");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5893.47,5278.09");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2380.03,5193.31");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3943.12,5244.80");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4895.59,5091.92");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=158.44,4985.47");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=627.32,5019.40");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=991.70,4963.50");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1526.70,4837.44");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3038.28,4947.29");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3476.47,4988.26");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4571.83,4940.86");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5972.53,4892.72");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2145.24,4749.35");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2563.05,4555.91");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5150.88,4732.44");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5595.23,4703.99");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=593.39,4480.06");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1100.16,4529.77");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1720.20,4386.47");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3165.98,4364.02");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3575.90,4521.22");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4244.86,4504.36");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4677.68,4468.06");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2063.90,4241.23");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2816.22,4073.29");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4999.02,4074.21");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5364.40,4048.67");
    }

}

