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

network DADCA_ACK_n100_u8_a8000m2_9
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
        @display("i=misc/sensor;p=77.91,7868.27");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=985.26,7966.76");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1696.85,7990.23");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2705.99,7853.90");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3559.49,7965.96");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4067.95,7940.07");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4763.72,7791.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=561.00,7595.95");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1149.10,7579.70");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1523.89,7625.31");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2063.29,7722.29");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3261.99,7592.51");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3686.05,7575.99");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4326.62,7611.37");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5739.93,7702.01");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=236.15,7268.83");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=856.36,7386.19");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1325.91,7268.20");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2485.05,7425.89");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4932.69,7350.22");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5924.67,7341.94");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=736.37,7056.81");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1996.57,7252.27");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2435.66,7060.64");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2795.73,7162.29");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3309.58,7143.85");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3987.42,7139.79");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4372.85,7074.84");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5574.81,7088.59");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=299.57,6852.28");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1219.60,6916.75");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1626.93,6772.00");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2025.08,6809.08");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5986.86,6861.14");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=597.43,6572.86");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2428.54,6651.90");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2791.07,6754.35");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3459.37,6540.10");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3940.74,6538.22");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4540.04,6675.84");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5382.33,6760.24");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5741.03,6574.13");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1292.05,6511.90");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=781.02,6167.16");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1700.84,6228.45");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2471.01,6155.79");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3036.17,6180.66");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3576.22,6104.85");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4087.09,6027.70");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4498.38,6147.94");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5531.49,6098.13");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1589.20,5780.09");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2070.09,5899.40");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2781.77,5902.85");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5035.89,6005.29");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5886.70,5965.62");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=962.95,5742.92");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2286.09,5590.51");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2954.08,5594.48");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3321.10,5705.87");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3653.23,5525.47");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4070.10,5632.52");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5241.96,5708.07");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5662.91,5655.78");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=502.93,5341.22");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2018.74,5356.94");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4692.05,5429.29");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5041.12,5339.25");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5966.02,5429.11");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1252.41,5194.75");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1650.61,5133.23");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2326.97,5057.12");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2669.71,5259.82");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3190.72,5154.96");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3681.40,5099.63");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=191.03,4980.27");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=878.77,4787.47");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2030.98,4801.39");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4297.21,5006.18");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4658.45,5012.42");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5029.08,4971.75");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5678.36,4964.17");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1569.03,4565.69");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2642.43,4776.66");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3012.58,4669.62");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3478.93,4718.03");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3864.70,4579.38");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4473.77,4553.95");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4829.35,4618.27");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5195.77,4568.26");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5923.20,4709.02");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=239.83,4402.92");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=767.71,4394.44");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2095.87,4425.47");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5476.08,4310.95");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1106.56,4207.05");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3014.12,4243.50");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3945.04,4196.33");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4393.22,4052.80");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5123.09,4197.80");
    }

}

