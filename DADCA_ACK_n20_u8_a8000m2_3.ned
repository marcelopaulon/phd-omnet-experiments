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

network DADCA_ACK_n20_u8_a8000m2_3
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
        @display("i=misc/sensor;p=378.92,7945.23");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1329.45,7955.37");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2300.07,7791.35");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3172.20,7978.43");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3791.15,7864.08");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5627.61,7885.02");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=464.70,7554.42");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=808.48,7645.67");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1938.69,7643.93");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2669.58,7641.03");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3134.29,7561.46");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4298.45,7635.92");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5227.27,7568.52");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5706.92,7542.67");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1225.62,7344.04");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1870.87,7284.68");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2233.71,7426.41");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4659.45,7370.59");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=607.42,7051.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1451.11,7015.39");
    }

}

