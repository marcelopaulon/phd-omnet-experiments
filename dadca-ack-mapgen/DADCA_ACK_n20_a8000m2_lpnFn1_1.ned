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

network DADCA_ACK_n20_a8000m2_lpnFn1_1
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
        @display("i=misc/sensor,p=96.48,198.29");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=156.89,153.12");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=164.32,226.31");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=168.94,89.67");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=299.86,63.41");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=338.55,14.59");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=345.96,189.62");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=362.55,125.11");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=266.75,210.73");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=263.86,147.70");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=27.96,252.59");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=133.84,314.68");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=80.63,370.14");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=34.30,329.74");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=152.43,418.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=202.92,368.38");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=227.66,457.34");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=180.42,495.35");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=289.93,323.29");
    }

}

