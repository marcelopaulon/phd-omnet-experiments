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

network DADCA_ACK_n20_a8000m2_2
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
        @display("i=misc/sensor;p=849.89,7929.48");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1410.46,7824.22");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2310.49,7833.17");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2803.33,7966.23");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4845.28,7968.28");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5649.26,7824.28");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=309.85,7655.01");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1811.41,7618.52");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2648.66,7543.19");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3306.71,7674.17");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4378.95,7707.33");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5218.60,7599.47");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=804.92,7343.44");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2205.71,7464.96");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3365.68,7304.79");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3814.75,7344.88");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4924.79,7280.74");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=111.06,7028.51");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1090.19,7127.46");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1627.39,7218.74");
    }

}

