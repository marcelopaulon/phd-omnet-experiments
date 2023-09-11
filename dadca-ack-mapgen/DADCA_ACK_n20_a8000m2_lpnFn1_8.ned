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

network DADCA_ACK_n20_a8000m2_lpnFn1_8
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=179.40,96.13");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=192.97,180.57");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=112.24,204.80");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=239.39,136.31");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=298.99,77.72");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=359.26,20.69");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=409.67,62.74");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=502.78,73.70");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=440.21,128.53");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=377.08,144.22");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=297.81,165.40");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=262.39,242.44");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=48.56,218.24");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=364.60,295.51");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=427.34,307.91");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=488.94,341.28");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=534.01,227.04");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=601.71,275.79");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=672.76,334.88");
    }

}

