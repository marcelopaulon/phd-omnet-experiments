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

network DADCA_ACK_n50_a8000m2_5
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
        @display("i=misc/sensor;p=1741.68,7982.48");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2585.52,7750.06");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4475.28,7795.94");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4907.63,7881.79");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1128.65,7698.72");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1498.49,7537.43");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2131.41,7596.91");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3066.06,7589.25");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3828.90,7577.94");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5176.00,7568.04");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4061.71,7263.61");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4551.70,7245.83");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=523.77,7221.48");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1167.07,7100.61");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1861.44,7108.36");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2892.36,7176.02");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3465.20,7155.17");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=479.86,6807.72");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2202.19,6945.56");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2580.51,6961.47");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4026.22,6893.02");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4742.13,6920.37");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=152.86,6607.53");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1542.37,6680.57");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2423.54,6610.69");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2798.62,6504.40");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3355.54,6595.74");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4024.05,6462.99");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=475.39,6352.66");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1000.95,6433.95");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2004.00,6441.89");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4463.57,6426.65");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5048.11,6390.03");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=134.24,6018.12");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=767.17,6070.80");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1315.61,6091.03");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3363.09,6156.25");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3930.52,6080.45");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4718.89,6024.28");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5552.07,6145.91");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=631.89,5691.09");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1800.06,5911.72");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2393.98,5925.64");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2782.42,5851.41");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3859.91,5691.75");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4345.75,5792.41");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5106.88,5689.05");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5785.70,5770.10");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=158.34,5545.95");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1340.10,5575.52");
    }

}

