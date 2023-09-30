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

network DADCA_ACK_n50_u2_a8000m2_2
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
        @display("i=misc/sensor;p=707.70,7896.50");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1810.75,7900.94");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2165.44,7804.36");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2974.80,7855.95");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3401.11,7886.14");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4081.77,7782.30");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4613.41,7925.55");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5111.06,7887.18");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=173.02,7723.08");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1161.05,7712.27");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3489.02,7539.05");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4917.85,7509.63");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5508.67,7712.95");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5952.46,7742.81");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=41.54,7278.60");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1535.85,7489.21");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2547.31,7349.35");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2908.89,7352.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4243.17,7469.02");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5290.68,7332.75");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=404.76,7179.52");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=885.72,7179.52");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1324.16,7107.01");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2094.45,7166.94");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3775.69,7246.52");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5997.86,7233.23");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3456.49,6842.06");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4465.03,6871.33");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4897.52,6779.83");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5571.23,6847.80");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=700.52,6673.92");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1389.42,6607.33");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2218.94,6713.17");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2875.89,6664.71");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3850.67,6682.90");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=51.43,6298.59");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1664.68,6288.45");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3443.79,6424.66");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4628.16,6389.25");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5219.27,6392.91");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5966.01,6454.73");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=782.94,6227.57");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1349.10,6081.96");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2326.27,6256.46");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4142.44,6228.97");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5445.00,6037.38");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=59.46,5940.04");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=960.93,5883.56");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2735.09,6012.24");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3303.76,5824.21");
    }

}

