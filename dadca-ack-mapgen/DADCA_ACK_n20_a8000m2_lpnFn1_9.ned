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

network DADCA_ACK_n20_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor,p=122.09,16.11");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=219.68,11.91");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=209.34,88.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=225.20,155.10");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=190.39,206.20");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=308.07,205.53");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=319.09,144.28");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=289.45,295.14");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=411.12,218.45");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=360.70,297.92");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=423.81,309.12");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=507.52,353.36");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=397.45,447.68");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=476.68,502.00");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=491.52,587.78");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=362.77,509.54");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=605.08,533.15");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=640.86,588.13");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=676.40,538.10");
    }

}

