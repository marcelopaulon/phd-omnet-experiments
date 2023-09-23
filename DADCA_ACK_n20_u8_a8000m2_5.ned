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

network DADCA_ACK_n20_u8_a8000m2_5
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
        @display("i=misc/sensor;p=242.32,7872.02");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1264.01,7928.71");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1789.33,7835.32");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2465.77,7840.17");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4841.95,7992.18");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=718.80,7655.69");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1429.50,7509.19");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3699.00,7681.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4354.68,7667.52");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=100.30,7460.37");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=475.71,7342.18");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1066.64,7282.81");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2189.54,7421.13");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2958.63,7471.76");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4756.70,7329.71");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5416.09,7503.12");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=16.00,7102.78");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=705.27,7053.93");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1831.09,7099.11");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2281.22,7022.17");
    }

}

