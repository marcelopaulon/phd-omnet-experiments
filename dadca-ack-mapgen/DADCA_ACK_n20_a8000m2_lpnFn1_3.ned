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

network DADCA_ACK_n20_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor,p=28.39,59.38");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=8.94,176.45");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=66.23,248.48");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=92.43,340.55");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=178.17,313.37");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=128.49,230.93");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=235.67,249.71");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=258.73,415.55");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=275.52,354.76");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=333.56,406.89");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=375.22,327.95");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=447.62,304.06");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=423.03,242.09");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=307.53,261.70");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=165.54,400.00");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=56.05,402.37");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=23.59,348.73");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=67.16,503.02");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=135.70,551.73");
    }

}

