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
        @display("i=misc/sensor,p=27.84,111.71");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=72.06,160.97");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=133.96,238.66");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=202.90,291.39");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=264.16,355.73");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=250.40,445.18");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=185.36,426.69");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=160.27,495.18");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=223.36,559.76");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=76.28,479.91");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=83.87,568.35");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=122.57,657.38");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=135.17,361.75");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=96.23,413.90");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=175.77,118.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=181.03,7.49");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=51.09,226.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=68.52,35.89");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=12.45,2.15");
    }

}

