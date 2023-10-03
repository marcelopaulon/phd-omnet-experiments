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

network DADCA_ACK_n100_u3_a8000m2_8
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
        @display("i=misc/sensor;p=73.84,7845.50");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=714.33,7905.85");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1149.03,7940.20");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1600.73,7956.63");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2689.24,7781.30");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3237.31,7771.06");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3614.71,7912.08");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4290.08,7783.55");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5863.42,7755.55");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2180.60,7740.96");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3706.91,7553.83");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4667.63,7551.59");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5476.92,7676.92");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=19.28,7491.15");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=577.72,7463.16");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=882.39,7285.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1263.16,7366.08");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1677.15,7460.14");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2619.11,7329.00");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3081.60,7409.54");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5013.00,7481.24");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5582.22,7333.26");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=265.18,7113.36");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1958.65,7175.41");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3776.10,7175.27");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4129.18,7200.79");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4482.27,7015.42");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5238.41,7097.62");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1617.89,6826.92");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2359.48,6780.74");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2848.26,6916.38");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3381.23,6908.05");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5803.05,6982.09");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=863.52,6567.21");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1258.14,6720.05");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2870.01,6529.21");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3803.56,6730.36");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4483.19,6656.98");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4942.30,6559.09");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5259.30,6713.13");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5711.19,6543.91");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=495.45,6455.59");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1162.42,6370.72");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1548.25,6282.34");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2189.29,6474.36");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3461.27,6317.88");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4076.62,6352.11");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=353.08,6074.41");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1016.96,6027.63");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2383.20,6087.26");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3190.35,6073.15");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4359.58,6034.14");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5545.92,6149.38");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1596.60,5902.07");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3673.22,5925.42");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5043.72,5901.81");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5925.09,5927.41");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=155.89,5731.11");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1325.52,5636.66");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3196.18,5591.50");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4014.12,5591.69");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4725.67,5543.80");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=40.62,5304.18");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=577.64,5497.16");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1020.40,5453.78");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2046.96,5510.97");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2538.33,5448.73");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5877.76,5452.29");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=443.95,5077.29");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=978.38,5040.37");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1500.32,5255.18");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1902.83,5120.41");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2279.77,5153.73");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3552.23,5086.42");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5218.58,5261.82");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=184.65,4815.43");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1396.14,4838.70");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2229.74,4799.76");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2605.05,4929.82");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3057.35,5015.92");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3932.28,4890.16");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4386.90,4978.06");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5611.89,5008.89");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=941.03,4674.14");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1832.43,4683.21");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3442.47,4626.39");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4668.60,4665.22");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5184.55,4590.84");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1.63,4321.64");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=377.24,4363.50");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1045.76,4308.98");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1709.50,4348.46");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2605.24,4371.15");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3054.35,4456.87");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4228.70,4369.47");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5678.80,4354.92");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2052.83,4193.40");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2880.60,4144.45");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3246.22,4065.19");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=10.96,3953.65");
    }

}

