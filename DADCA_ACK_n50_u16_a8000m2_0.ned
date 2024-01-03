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

network DADCA_ACK_n50_u16_a8000m2_0
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
        @display("i=misc/sensor;p=409.42,7820.03");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=940.20,7938.18");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1736.91,7867.17");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2404.18,7782.48");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3702.64,7923.33");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4523.61,7840.44");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5076.50,7900.27");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1235.43,7718.04");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3216.37,7687.87");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3955.58,7628.95");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5451.66,7509.29");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=189.40,7483.76");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=618.87,7401.48");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=985.11,7471.70");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1555.92,7420.24");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2108.74,7488.20");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2667.99,7300.06");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3020.15,7291.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4536.37,7293.35");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4929.10,7434.15");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1166.22,7161.19");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1891.47,7075.94");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3614.11,7252.05");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4162.91,7228.44");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5187.95,7131.82");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5842.62,7102.78");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=316.92,6851.13");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=655.70,6994.89");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2179.34,6816.45");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3215.22,6972.21");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4375.90,6891.11");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=825.48,6575.03");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1174.74,6690.02");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1583.40,6655.22");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3597.40,6699.81");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4046.12,6518.26");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5016.04,6715.91");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5593.07,6562.50");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=150.20,6424.96");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2176.27,6408.63");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2863.05,6381.68");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4615.92,6431.23");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1841.00,6241.80");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3088.91,6056.93");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4181.00,6027.02");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5177.64,6207.50");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5975.03,6085.88");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=161.87,5887.67");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=997.33,5984.84");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1374.83,6008.77");
    }

}

