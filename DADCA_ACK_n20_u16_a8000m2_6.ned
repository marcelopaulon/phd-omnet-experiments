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

network DADCA_ACK_n20_u16_a8000m2_6
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
        @display("i=misc/sensor;p=200.26,7798.59");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=540.23,7913.75");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1533.27,7917.49");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2493.55,7890.04");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3223.82,7961.41");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3727.33,7842.05");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=833.85,7654.20");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1430.86,7524.69");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1941.16,7663.13");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2972.00,7689.02");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3439.03,7588.67");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4616.96,7726.30");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5432.89,7573.19");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2326.58,7364.48");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2777.77,7342.50");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3741.71,7302.78");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4366.74,7326.55");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4768.36,7407.43");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5131.63,7388.28");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=24.17,7043.84");
    }

}

