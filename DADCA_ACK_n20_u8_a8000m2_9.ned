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

network DADCA_ACK_n20_u8_a8000m2_9
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
        @display("i=misc/sensor;p=1022.25,7793.55");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1864.92,7991.74");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2422.10,7925.90");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3057.68,7991.12");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4107.89,7880.04");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4876.74,7844.44");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5242.54,7792.86");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=130.71,7606.73");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1300.48,7553.13");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1835.26,7592.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3585.62,7604.80");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4490.85,7704.17");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5932.23,7734.93");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=798.25,7369.97");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2345.15,7412.73");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3037.78,7431.00");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4036.42,7321.23");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4496.73,7348.73");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5024.02,7308.61");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5677.41,7293.21");
    }

}

