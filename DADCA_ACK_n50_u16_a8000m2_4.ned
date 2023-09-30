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

network DADCA_ACK_n50_u16_a8000m2_4
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
        @display("i=misc/sensor;p=779.45,7836.37");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2420.76,7962.00");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3540.97,7919.72");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4362.62,7977.75");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4824.80,7880.39");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5548.13,7842.33");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5933.21,7986.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=207.24,7537.32");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1712.98,7740.85");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2216.60,7616.29");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2755.06,7715.38");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3269.41,7562.57");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3843.89,7664.32");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4322.76,7592.22");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4759.54,7512.09");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5156.34,7609.75");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=664.98,7387.66");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1479.07,7379.12");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1847.27,7314.89");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2632.68,7287.42");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5602.71,7480.35");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5984.69,7472.66");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=277.98,7116.29");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1002.48,7187.54");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3535.05,7229.93");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3961.51,7054.11");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4700.83,7041.74");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5075.23,7107.23");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5949.56,7055.13");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=811.47,6769.14");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1267.01,6956.91");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1694.69,6799.95");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2755.41,6942.36");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3267.71,6999.22");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4400.62,6822.19");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5603.30,6789.08");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2085.04,6708.91");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3440.68,6621.91");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3918.55,6686.00");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5979.96,6617.84");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=465.64,6477.86");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1019.49,6472.83");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2336.30,6437.86");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2947.76,6506.45");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5073.93,6415.22");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=553.06,6059.58");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1219.58,6074.88");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1772.62,6223.04");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3277.49,6099.42");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4079.98,6241.94");
    }

}

