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

network DADCA_ACK_n20_a8000m2_lpnFn1_6
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
        @display("i=misc/sensor,p=70.19,31.25");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=143.66,177.04");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=186.74,229.62");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=218.79,161.88");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=291.93,168.91");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=279.48,86.35");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=344.03,91.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=406.60,149.73");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=439.73,208.84");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=519.53,75.73");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=214.91,287.73");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=274.29,357.25");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=305.16,438.49");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=262.80,493.47");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=317.77,538.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=154.82,314.14");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=193.80,399.08");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=82.00,375.13");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=87.88,442.58");
    }

}

