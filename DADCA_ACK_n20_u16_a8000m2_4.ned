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

network DADCA_ACK_n20_u16_a8000m2_4
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
        @display("i=misc/sensor;p=485.82,7831.54");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1165.15,7981.80");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1515.12,7968.20");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2103.32,7772.71");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2665.27,7836.35");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4732.62,7893.73");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5298.40,7974.88");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5881.57,7754.21");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=100.58,7559.40");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3296.89,7666.78");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3903.80,7716.78");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5333.19,7569.26");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=823.63,7475.37");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1176.16,7487.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1652.52,7342.88");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2228.58,7396.78");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2694.11,7427.79");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4176.60,7460.07");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4518.71,7269.14");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5051.85,7293.48");
    }

}

