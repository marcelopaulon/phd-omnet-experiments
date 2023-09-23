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

network DADCA_ACK_n20_u16_a8000m2_2
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
        @display("i=misc/sensor;p=498.90,7960.12");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1456.22,7966.28");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2102.64,7843.70");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2481.45,7767.86");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3079.62,7910.54");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4484.30,7836.88");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4819.94,7950.39");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1049.75,7548.62");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2798.72,7550.26");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3856.38,7664.46");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4858.89,7593.78");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5220.90,7734.06");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5635.25,7629.35");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5987.18,7589.38");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=178.11,7442.42");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=744.82,7333.01");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1918.01,7453.36");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2507.47,7296.73");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3323.91,7456.55");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4381.03,7362.89");
    }

}

