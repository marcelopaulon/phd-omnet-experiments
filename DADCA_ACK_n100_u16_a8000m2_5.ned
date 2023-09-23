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

network DADCA_ACK_n100_u16_a8000m2_5
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
        @display("i=misc/sensor;p=35.51,7939.24");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=626.87,7937.61");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1254.44,7815.08");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1928.76,7816.86");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2765.17,7825.00");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4315.06,7962.76");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4904.02,7970.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5279.30,7905.03");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3329.38,7700.74");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3777.34,7725.27");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=358.63,7460.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=800.74,7311.52");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1299.49,7304.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1613.05,7473.08");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1929.74,7310.80");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2316.77,7287.94");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3022.04,7414.92");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3614.58,7337.07");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4097.89,7470.12");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4995.67,7467.31");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5593.55,7466.89");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2708.27,7144.08");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3335.63,7125.27");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3921.35,7053.35");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4277.70,7032.30");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4647.96,7103.67");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=174.89,6888.60");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=657.25,6788.48");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1457.33,6824.80");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1929.07,6955.21");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2459.98,6804.82");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5415.63,6860.73");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5869.15,6789.14");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=420.79,6515.16");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2119.55,6610.46");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2821.05,6671.85");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3430.98,6584.87");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4505.50,6732.70");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4919.30,6675.04");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=17.79,6422.41");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=751.48,6312.53");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1234.77,6431.53");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1733.43,6315.91");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2449.84,6312.02");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2983.64,6347.61");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3818.74,6413.05");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5899.78,6395.76");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=410.43,6124.03");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4292.31,6145.30");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5168.61,6247.30");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5585.70,6073.91");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=966.71,5960.97");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1317.50,5991.81");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2090.97,5892.57");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3306.25,5983.81");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3743.62,5964.41");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5238.42,5829.98");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=640.21,5683.77");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1775.80,5598.31");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2496.42,5771.76");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3128.16,5604.97");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4798.50,5761.39");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5642.29,5726.06");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=334.60,5480.27");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=722.31,5326.64");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1278.09,5446.39");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3677.39,5516.66");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4123.13,5508.72");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4523.64,5503.45");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5128.71,5357.17");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5624.58,5351.35");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2361.76,5230.69");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2907.74,5271.78");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3476.85,5109.81");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4714.79,5169.00");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5976.58,5041.56");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=289.87,5017.25");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1133.95,4931.61");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1501.31,4882.82");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3230.88,4818.18");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3832.16,4986.52");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4782.44,4822.67");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5129.82,4990.87");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=45.36,4672.94");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=716.81,4649.62");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2768.03,4634.76");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4378.10,4629.38");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5513.27,4677.71");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1090.80,4485.28");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1853.42,4353.24");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2472.58,4384.72");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2963.12,4327.61");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3347.53,4456.61");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3922.29,4373.28");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=310.72,4246.45");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1026.49,4125.89");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1500.52,4216.78");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2718.48,4056.84");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3345.31,4090.20");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4756.57,4225.69");
    }

}

