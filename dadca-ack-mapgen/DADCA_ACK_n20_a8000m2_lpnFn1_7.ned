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

network DADCA_ACK_n20_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor,p=70.96,47.28");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=31.52,105.26");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=122.65,184.23");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=65.89,208.27");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=208.25,73.43");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=147.73,46.87");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=212.89,198.07");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=168.24,266.91");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=98.87,273.04");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=147.22,349.26");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=241.68,329.85");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=209.25,418.75");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=34.61,355.75");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=6.77,476.21");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=87.71,472.36");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=86.87,533.99");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=43.24,605.37");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=121.70,622.24");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=182.09,596.19");
    }

}

